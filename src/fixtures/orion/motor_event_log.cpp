#include "fixtures/orion/motor_event_log.h"

#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
#include <Arduino.h>
#include <LittleFS.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

namespace OrionMotorEvents {
namespace {
constexpr uint16_t RecordsPerFile = Capacity / 2;
constexpr uint8_t WriteBatch = 8;
constexpr uint32_t FlushIntervalMs = 5000;
constexpr const char* Paths[] = {"/orion-motor-a.v1", "/orion-motor-b.v1"};

struct DiskRecord {
    Event event;
    uint32_t checksum;
    uint32_t reserved;
};
static_assert(sizeof(Event) == 24 && sizeof(DiskRecord) == 32, "Motor log disk format changed");

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Event events[Capacity]{};
uint16_t head = 0;
uint16_t used = 0;
uint64_t nextSequence = 1;
uint64_t persistedThrough = 0;
uint32_t bootId = 0;
bool storageAvailable = false;
bool storageWriteFailed = false;

// The setup/loop task alone owns the files and these rotation counters.
uint8_t activeFile = 0;
uint16_t activeRecords = 0;
uint32_t lastFlush = 0;

uint32_t checksum(const Event& event) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&event);
    uint32_t result = 2166136261u;
    for (size_t i = 0; i < sizeof(event); ++i) result = (result ^ bytes[i]) * 16777619u;
    return result;
}

void appendToMemory(const Event& event) {
    events[head] = event;
    head = (head + 1) % Capacity;
    if (used < Capacity) ++used;
}

struct Segment {
    uint64_t last = 0;
    uint16_t records = 0;
    bool appendable = true;
};

Segment readSegment(uint8_t index, bool load) {
    Segment segment;
    File file = LittleFS.open(Paths[index], FILE_READ);
    if (!file) return segment;
    const size_t size = file.size();
    while (segment.records < RecordsPerFile &&
           (segment.records + 1u) * sizeof(DiskRecord) <= size) {
        DiskRecord item;
        if (file.read(reinterpret_cast<uint8_t*>(&item), sizeof(item)) != sizeof(item) ||
            item.event.sequence <= segment.last || item.reserved != 0 ||
            checksum(item.event) != item.checksum) break;
        segment.last = item.event.sequence;
        ++segment.records;
        if (load && item.event.sequence >= nextSequence) {
            appendToMemory(item.event);
            nextSequence = item.event.sequence + 1;
        }
    }
    segment.appendable = size == segment.records * sizeof(DiskRecord);
    return segment;
}
}

void begin() {
    const bool mounted = LittleFS.begin(false);
    if (mounted) {
        const Segment a = readSegment(0, false);
        const Segment b = readSegment(1, false);
        activeFile = b.last > a.last ? 1 : 0;
        readSegment(activeFile ^ 1, true);
        const Segment active = readSegment(activeFile, true);
        activeRecords = active.appendable ? active.records : RecordsPerFile;
        persistedThrough = active.last;
        nextSequence = persistedThrough + 1;
    }
    storageAvailable = mounted;
    bootId = esp_random();
    record(Kind::BOOT, static_cast<uint16_t>(esp_reset_reason()));
}

void record(Kind kind, uint16_t detail, uint8_t previous) {
    const uint64_t uptime = esp_timer_get_time() / 1000ULL;
    portENTER_CRITICAL(&mux);
    appendToMemory({nextSequence++, uptime, bootId, detail, kind, previous});
    portEXIT_CRITICAL(&mux);
}

void service() {
    if (!storageAvailable) return;
    const uint32_t now = millis();
    if (now - lastFlush < FlushIntervalMs) return;
    lastFlush = now;
    Event batch[WriteBatch];
    uint8_t count = 0;
    portENTER_CRITICAL(&mux);
    const uint16_t oldest = (head + Capacity - used) % Capacity;
    for (uint16_t i = 0; i < used && count < WriteBatch; ++i) {
        const Event& event = events[(oldest + i) % Capacity];
        if (event.sequence > persistedThrough) batch[count++] = event;
    }
    portEXIT_CRITICAL(&mux);
    if (!count) return;

    const bool rotate = activeRecords == RecordsPerFile;
    const uint8_t index = rotate ? activeFile ^ 1 : activeFile;
    File file = LittleFS.open(Paths[index], rotate ? FILE_WRITE : FILE_APPEND);
    if (!file) {
        portENTER_CRITICAL(&mux);
        storageWriteFailed = true;
        portEXIT_CRITICAL(&mux);
        return;
    }
    activeFile = index;
    if (rotate) activeRecords = 0;
    uint64_t last = 0;
    bool writeFailed = false;
    for (uint8_t i = 0; i < count && activeRecords < RecordsPerFile; ++i) {
        const DiskRecord item{batch[i], checksum(batch[i]), 0};
        if (file.write(reinterpret_cast<const uint8_t*>(&item), sizeof(item)) != sizeof(item)) {
            // Keep complete records and rotate on the next flush.
            writeFailed = true;
            break;
        }
        last = item.event.sequence;
        ++activeRecords;
    }
    file.close();
    const Segment verified = readSegment(activeFile, false);
    activeRecords = !writeFailed && verified.appendable ? verified.records : RecordsPerFile;
    portENTER_CRITICAL(&mux);
    storageWriteFailed = writeFailed || verified.last != last;
    if (verified.last > persistedThrough) persistedThrough = verified.last;
    portEXIT_CRITICAL(&mux);
}

void page(uint64_t before, Page& out) {
    portENTER_CRITICAL(&mux);
    out.count = 0;
    out.bootId = bootId;
    out.storageAvailable = storageAvailable;
    out.storageWriteFailed = storageWriteFailed;
    out.persistedThrough = persistedThrough;
    out.oldestSequence = used ? events[(head + Capacity - used) % Capacity].sequence : 0;
    out.newestSequence = nextSequence - 1;
    for (uint16_t i = 0; i < used && out.count < PageSize; ++i) {
        const Event& event = events[(head + Capacity - 1 - i) % Capacity];
        if (!before || event.sequence < before) out.events[out.count++] = event;
    }
    portEXIT_CRITICAL(&mux);
}

const char* eventName(Kind kind) {
    switch (kind) {
        case Kind::BOOT: return "boot";
        case Kind::TMC_RESET: return "tmc_reset";
        case Kind::TMC_RESET_AT_BOOT: return "tmc_reset_at_boot";
        case Kind::MOTOR_FAULT: return "motor_fault";
        case Kind::MOTOR_FAULT_CLEAR: return "motor_fault_clear";
        case Kind::THERMAL_WARNING: return "thermal_warning";
        case Kind::THERMAL_SHUTDOWN: return "thermal_shutdown";
        case Kind::THERMAL_CLEAR: return "thermal_clear";
    }
    return "unknown";
}
}
#endif
