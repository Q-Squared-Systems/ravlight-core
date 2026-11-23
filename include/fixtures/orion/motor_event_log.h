#pragma once

#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
#include <stdint.h>

namespace OrionMotorEvents {
enum class Kind : uint8_t {
    BOOT = 1,
    TMC_RESET,
    TMC_RESET_AT_BOOT,
    MOTOR_FAULT,
    MOTOR_FAULT_CLEAR,
    THERMAL_WARNING,
    THERMAL_SHUTDOWN,
    THERMAL_CLEAR,
};

struct Event {
    uint64_t sequence;
    uint64_t uptimeMs;
    uint32_t bootId;
    uint16_t detail;
    Kind kind;
    uint8_t previous;
};

constexpr uint16_t Capacity = 128;
constexpr uint8_t PageSize = 24;

struct Page {
    Event events[PageSize];
    uint64_t oldestSequence;
    uint64_t newestSequence;
    uint64_t persistedThrough;
    uint32_t bootId;
    uint8_t count;
    bool storageAvailable;
    bool storageWriteFailed;
};

void begin();
void record(Kind kind, uint16_t detail = 0, uint8_t previous = 0);
void service();
void page(uint64_t before, Page& out);
const char* eventName(Kind kind);
}
#endif
