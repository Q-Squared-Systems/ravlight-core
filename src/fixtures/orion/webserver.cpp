#ifdef RAVLIGHT_FIXTURE_ORION
#include "fixture_webserver.h"
#include "fixtures/orion/fixture.h"
#include "fixtures/orion/motor_event_log.h"
#include <esp_timer.h>
#include <errno.h>
#include "core/motor/IMotorDriver.h"
#include "core/motor/backends/Tmc2209LocalDriver.h"
#include "config.h"
#include "dmx_manager.h"
#include <ESPAsyncWebServer.h>
#include <WebResponseImpl.h>
#include <ArduinoJson.h>
#include <esp_log.h>
#include <memory>
#include <new>

static const char* TAG = "ORION_WEB";

class SgLogResponse final : public AsyncAbstractResponse {
public:
    using Entry = Tmc2209LocalDriver::SgLogEntry;

    SgLogResponse(std::unique_ptr<Entry[]> entries, uint16_t count)
        : _entries(std::move(entries)), _count(count) {
        _code = 200;
        _contentType = "application/json";
        _contentLength = 2;
        for (uint16_t i = 0; i < _count; ++i)
            _contentLength += formatRow(nullptr, 0, i);
    }

    bool _sourceValid() const override { return true; }

    size_t _fillBuffer(uint8_t* out, size_t capacity) override {
        size_t written = 0;
        while (written < capacity) {
            if (_offset == _length) {
                if (_closed) break;
                _offset = 0;
                if (!_opened) {
                    _row[0] = '[';
                    _length = 1;
                    _opened = true;
                } else if (_next < _count) {
                    _length = formatRow(_row, sizeof(_row), _next++);
                } else {
                    _row[0] = ']';
                    _length = 1;
                    _closed = true;
                }
            }
            const size_t bytes = min(_length - _offset, capacity - written);
            memcpy(out + written, _row + _offset, bytes);
            written += bytes;
            _offset += bytes;
        }
        return written;
    }

private:
    size_t formatRow(char* out, size_t size, uint16_t i) const {
        const Entry& e = _entries[i];
        return snprintf(out, size,
            "%s{\"t\":%lu,\"sg\":%u,\"spd\":%ld,\"st\":%u,\"dg\":%u,\"gr\":%u,\"ig\":%u}",
            i ? "," : "", static_cast<unsigned long>(e.t), static_cast<unsigned>(e.sg),
            static_cast<long>(e.spd10) * 10, static_cast<unsigned>(e.state),
            (e.flags >> 0) & 1u, (e.flags >> 1) & 1u, (e.flags >> 2) & 1u);
    }

    std::unique_ptr<Entry[]> _entries;
    const uint16_t _count;
    uint16_t _next = 0;
    char _row[104]{};
    size_t _offset = 0;
    size_t _length = 0;
    bool _opened = false;
    bool _closed = false;
};

static bool parseSgLogParam(AsyncWebServerRequest* req, const char* key, uint64_t& result) {
    if (!req->hasParam(key)) return true;
    const String& value = req->getParam(key)->value();
    char* end = nullptr;
    errno = 0;
    result = strtoull(value.c_str(), &end, 10);
    return result && value[0] >= '0' && value[0] <= '9' && !*end && errno != ERANGE;
}

#ifdef ORION_HAS_LED
static long orionFormLedInt(AsyncWebServerRequest* request, const char* base, uint8_t index, long fallback) {
    const String key = String(base) + index;
    return request->hasParam(key, true) ? request->getParam(key, true)->value().toInt() : fallback;
}

static void readOrionFormLedPatch(AsyncWebServerRequest* request, uint8_t index, led_output_cfg_t& output) {
    output.protocol = static_cast<led_protocol_t>(orionFormLedInt(request, "orionLedProto", index, output.protocol));
    output.pixel_count = orionFormLedInt(request, "orionLedCount", index, output.pixel_count);
    output.universe_start = orionFormLedInt(request, "orionLedUniv", index, output.universe_start);
    output.dmx_start = orionFormLedInt(request, "orionLedCh", index, output.dmx_start);
    output.grouping = orionFormLedInt(request, "orionLedGroup", index, output.grouping);
    if (!output.grouping) output.grouping = 1;
}

#ifdef RAVLIGHT_ORION_COMPACT_DMX_POOL
bool reserveOrionFormDmxPatch(AsyncWebServerRequest* request, uint16_t motorUniverse) {
    led_output_cfg_t outputs[HW_LED_OUTPUT_COUNT];
    for (uint8_t i = 0; i < HW_LED_OUTPUT_COUNT; ++i) {
        outputs[i] = orionConfig.ledOutputs[i];
        readOrionFormLedPatch(request, i, outputs[i]);
    }
    return orionReserveDmxPatch(motorUniverse, outputs, HW_LED_OUTPUT_COUNT);
}
#endif
#endif

static bool rejectDuringPassiveBrakeTest(AsyncWebServerRequest* request,
                                         bool allowHomedWebPosition = false) {
#ifdef HW_MOTOR_PASSIVE_BRAKE_TEST
    if (orionPassiveBrakeTestBusy(allowHomedWebPosition)) {
        request->send(409, "text/plain", "Motor controls are blocked by braking or recovery");
        return true;
    }
#else
    (void)request;
    (void)allowHomedWebPosition;
#endif
    return false;
}

void handleFixtureSaveParams(AsyncWebServerRequest* request, bool& needsRestart) {
    // Motor params apply live; LED pixel-count / protocol changes need a restart
    // (RMT channels are sized at init time).
    bool changed = false;

    auto setU8 = [&](const char* name, uint8_t& field) {
        if (request->hasParam(name, true)) {
            uint8_t v = (uint8_t)request->getParam(name, true)->value().toInt();
            if (v != field) { field = v; changed = true; }
        }
    };
    auto setU16 = [&](const char* name, uint16_t& field) {
        if (request->hasParam(name, true)) {
            uint16_t v = (uint16_t)request->getParam(name, true)->value().toInt();
            if (v != field) { field = v; changed = true; }
        }
    };

    // cm-based fields → converted to steps internally. Float value in form, integer step in config.
    auto setCmToStepsI = [&](const char* name, int32_t& field) {
        if (request->hasParam(name, true)) {
            float cm = request->getParam(name, true)->value().toFloat();
            int32_t v = (int32_t)(cm * orionStepsPerCm());
            if (v != field) { field = v; changed = true; }
        }
    };
    auto setCmToStepsU = [&](const char* name, uint32_t& field) {
        if (request->hasParam(name, true)) {
            float cm = request->getParam(name, true)->value().toFloat();
            if (cm < 0) cm = 0;
            uint32_t v = (uint32_t)(cm * orionStepsPerCm());
            if (v != field) { field = v; changed = true; }
        }
    };

    setU8 ("personality",     orionConfig.personality);
    setU16("positionStart",   orionConfig.positionStart);
    setU16("controlStart",    orionConfig.controlStart);
    setCmToStepsI("downCm", orionConfig.downPosition);
    setCmToStepsI("upCm",   orionConfig.upPosition);
    setCmToStepsU("maxSpeedCm", orionConfig.maxSpeed);
    setCmToStepsU("maxAccelCm", orionConfig.maxAccel);
    setCmToStepsU("jogSpeedCm", orionConfig.jogSpeed);
    setU16("runCurrentMa",  orionConfig.runCurrentMa);
    setU16("holdCurrentMa", orionConfig.holdCurrentMa);
    // Checkbox: present = true, absent = false (standard HTML form behaviour)
    {
        bool v = request->hasParam("autoRehomeOnStall", true);
        if (v != orionConfig.autoRehomeOnStall) { orionConfig.autoRehomeOnStall = v; changed = true; }
    }
    setU8 ("dmxWatchdogAction", orionConfig.dmxWatchdogAction);
    setU16("drumDiaMm",     orionConfig.drumDiameterMm);
    setU16("motorStepsRev", orionConfig.motorStepsPerRev);
    if (request->hasParam("gearRatio", true)) {
        float g = request->getParam("gearRatio", true)->value().toFloat();
        if (g >= 0.01f && g != orionConfig.gearRatio) { orionConfig.gearRatio = g; changed = true; }
    }

    if (request->hasParam("homingDirection", true)) {
        int8_t v = (int8_t)request->getParam("homingDirection", true)->value().toInt();
        if (v != orionConfig.homingDirection) {
            orionConfig.homingDirection = (v >= 0) ? 1 : -1;
            changed = true;
            orionApplyHomingConfig();   // propagate to running driver
        }
    }

#ifdef ORION_HAS_LED
    for (int i = 0; i < HW_LED_OUTPUT_COUNT; i++) {
        led_output_cfg_t& o = orionConfig.ledOutputs[i];
        String s = String(i);
        const auto previous = o;
        readOrionFormLedPatch(request, i, o);
        if (o.protocol != previous.protocol || o.pixel_count != previous.pixel_count) {
            needsRestart = true;
            changed = true;
        }
        uint8_t  newBri  = orionFormLedInt(request, "orionLedBri", i, o.brightness);
        uint8_t  newInv  = request->hasParam("orionLedInv" + s, true) ? 1 : 0;
        if (o.universe_start != previous.universe_start || o.dmx_start != previous.dmx_start ||
            o.grouping != previous.grouping || newBri != o.brightness || newInv != o.invert) changed = true;
        o.brightness     = newBri;
        o.invert         = newInv;

        String ok = "orionLedOrder" + s;
        if (request->hasParam(ok, true)) {
            String v = request->getParam(ok, true)->value();
            v.toUpperCase();
            color_order_from_str(v.c_str(), o.color_order);
            changed = true;
        }
    }
#endif

    // Apply live: soft limits + motor currents take effect immediately.
    if (changed) {
        orionApplySoftLimitsExternal();
        orionApplyMotorCurrents();
    }
}

// ── Fixture-specific routes ─────────────────────────────────────────────────

static const char* stateName(MotorState s) {
    switch (s) {
        case MotorState::IDLE:       return "idle";
        case MotorState::HOMING:     return "homing";
        case MotorState::MOVING:     return "moving";
        case MotorState::JOGGING:    return "jogging";
        case MotorState::FAULT:      return "fault";
        case MotorState::DRIVER_OFF: return "driver_off";
    }
    return "unknown";
}

#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
static void addRegisterRead(JsonObject object, const char* name,
                            Tmc2209LocalDriver::RegisterRead sample) {
    if (sample.valid) object[name] = sample.value;
    else object[name] = nullptr;
}

static const char* checkRegister(Tmc2209LocalDriver::RegisterRead actual, uint32_t expected) {
    if (!actual.valid) return "read_failed";
    return actual.value == expected ? "match" : "mismatch";
}

static void addProfileTrace(JsonObject object, const Tmc2209LocalDriver::ProfileTrace& trace) {
    object["result"] = trace.result;
    object["attempts"] = trace.attempts;
    if (!trace.attempts) return;

    JsonObject uart = object.createNestedObject("ifcnt");
    addRegisterRead(uart, "before", trace.ifcntBefore);
    addRegisterRead(uart, "after", trace.ifcntAfter);
    uart["expectedDelta"] = trace.expectedWrites;
    if (trace.ifcntBefore.valid && trace.ifcntAfter.valid)
        uart["actualDelta"] = static_cast<uint8_t>(trace.ifcntAfter.value - trace.ifcntBefore.value);

    if (trace.targetKnown) {
        JsonObject target = object.createNestedObject("written");
        target["GCONF"] = trace.targetGconf;
        target["PWMCONF"] = trace.targetPwmconf;
        if (trace.expectedWrites == 3) target["IHOLD_IRUN"] = trace.targetCurrents;
    }

    const auto& registers = trace.readback;
    JsonObject raw = object.createNestedObject("readback");
    raw["uptimeMs"] = registers.uptimeMs;
    addRegisterRead(raw, "GCONF", registers.gconf);
    addRegisterRead(raw, "PWMCONF", registers.pwmconf);
    addRegisterRead(raw, "IOIN", registers.ioin);
    addRegisterRead(raw, "DRV_STATUS", registers.drvStatus);
    addRegisterRead(raw, "TSTEP", registers.tstep);

    JsonObject checks = object.createNestedObject("checks");
    if (trace.targetKnown) {
        checks["expectedEffectiveGCONF"] = trace.expectedEffectiveGconf;
        checks["GCONF"] = checkRegister(registers.gconf, trace.expectedEffectiveGconf);
        checks["PWMCONF"] = checkRegister(registers.pwmconf, trace.targetPwmconf);
    }
    if (registers.ioin.valid) checks["spreadPinHigh"] = (registers.ioin.value & (1u << 8)) != 0;
    if (registers.gconf.valid) checks["spreadCycleSelected"] = (registers.gconf.value & (1u << 2)) != 0;
    checks["activeChopper"] = !registers.drvStatus.valid ? "read_failed" :
        (registers.drvStatus.value & (1u << 30)) ? "StealthChop" : "SpreadCycle";
}
#endif

void registerFixtureRoutes(AsyncWebServer& server) {
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    server.on("/api/motor-events", HTTP_GET, [](AsyncWebServerRequest* req) {
        uint64_t before = 0;
        if (req->hasParam("before")) {
            const String& value = req->getParam("before")->value();
            char* end = nullptr;
            errno = 0;
            before = strtoull(value.c_str(), &end, 10);
            if (!before || value[0] < '0' || value[0] > '9' || *end || errno == ERANGE) {
                req->send(400, "text/plain", "before must be a positive event sequence");
                return;
            }
        }
        using namespace OrionMotorEvents;
        Page snapshot;
        page(before, snapshot);
        DynamicJsonDocument doc(4096);
        doc["bootId"] = snapshot.bootId;
        doc["uptimeMs"] = static_cast<uint64_t>(esp_timer_get_time() / 1000ULL);
        doc["capacity"] = Capacity;
        doc["storageAvailable"] = snapshot.storageAvailable;
        doc["storageWriteFailed"] = snapshot.storageWriteFailed;
        doc["oldestSequence"] = snapshot.oldestSequence;
        doc["newestSequence"] = snapshot.newestSequence;
        doc["persistedThrough"] = snapshot.persistedThrough;
        JsonArray items = doc.createNestedArray("events");
        for (uint8_t i = 0; i < snapshot.count; ++i) {
            const Event& event = snapshot.events[i];
            JsonObject item = items.createNestedObject();
            item["seq"] = event.sequence;
            item["bootId"] = event.bootId;
            item["uptimeMs"] = event.uptimeMs;
            item["type"] = eventName(event.kind);
            item["detail"] = event.detail;
            if (event.kind >= Kind::MOTOR_FAULT) item["previous"] = event.previous;
        }
        if (snapshot.count && snapshot.events[snapshot.count - 1].sequence > snapshot.oldestSequence)
            doc["nextBefore"] = snapshot.events[snapshot.count - 1].sequence;
        if (doc.overflowed()) {
            req->send(503, "text/plain", "Motor event response buffer unavailable");
            return;
        }
        String out;
        const size_t length = measureJson(doc);
        if (!out.reserve(length) || serializeJson(doc, out) != length) {
            req->send(503, "text/plain", "Motor event response buffer unavailable");
            return;
        }
        auto* response = req->beginResponse(200, "application/json", out);
        response->addHeader("Cache-Control", "no-store");
        req->send(response);
    });
#endif
    // GET /motorstatus — JSON snapshot for UI polling
    server.on("/motorstatus", HTTP_GET, [](AsyncWebServerRequest* req) {
        IMotorDriver* drv = orionGetDriver();
        StaticJsonDocument<768> doc;

        if (!drv) {
            doc["available"] = false;
            doc["error"]     = "driver not initialised";
        } else {
            MotorStatus s = drv->getStatus();
            doc["available"]    = true;
            doc["state"]        = stateName(s.state);
            doc["faultFlags"]   = s.fault_flags;
            doc["sgResult"]     = s.sg_result;
            doc["driverTemp"]   = s.driver_temp;
            doc["homed"]        = s.homed;
            doc["busy"]         = drv->isBusy();
            // cm-converted values for UI (cm-native)
            const float spc = orionStepsPerCm();
            doc["positionCm"] = (float)s.position / spc;
            doc["speedCm"]    = s.speed / spc;
            doc["downCm"]     = (float)orionConfig.downPosition / spc;
            doc["upCm"]       = (float)orionConfig.upPosition / spc;
            doc["dmxActive"]  = dmxIsActive();
            doc["dmxEnable"]  = orionDmxLastEnable();
            doc["override"]   = orionManualOverride();
            doc["operSgthrs"] = orionConfig.operSgthrs;
            doc["homingSpeed"]= ORION_HOMING_SPEED;
            doc["manualMode"] = orionConfig.manualMode;
#ifdef RAVLIGHT_TMC_STEALTHCHOP_TRIAL
            if (auto* tmc = orionGetTmcDriver()) {
                doc["chopperModeAtIdle"] = tmc->chopperModeAtIdle();
#ifdef RAVLIGHT_TMC_COOLSTEP_TRIAL
                doc["coolStepConfigured"] = tmc->coolStepConfigured();
#endif
            }
#endif
#ifdef HW_MOTOR_PASSIVE_BRAKE_TEST
            doc["passiveBrakeTest"] = orionPassiveBrakeTestStatus();
#endif
#ifdef RAVLIGHT_ORION_GRAVITY_RECOVERY
            doc["gravityRecovery"] = orionGravityRecoveryStatus();
            if (const char* failure = orionGravityRecoveryFailure())
                doc["gravityRecoveryFailure"] = failure;
#endif
        }

        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
    server.on("/api/tmc-recovery", HTTP_GET, [](AsyncWebServerRequest* req) {
        auto* tmc = orionGetTmcDriver();
        if (!tmc) {
            req->send(503, "text/plain", "Driver unavailable");
            return;
        }
        const auto traces = tmc->profileDiagnostics();
        StaticJsonDocument<2048> doc;
        addProfileTrace(doc.createNestedObject("bootProfile"), traces.bootProfile);
        addProfileTrace(doc.createNestedObject("recoveryRestore"), traces.recoveryRestore);
        if (doc.overflowed()) {
            req->send(503, "text/plain", "Diagnostic buffer unavailable");
            return;
        }
        String out;
        const size_t length = measureJson(doc);
        if (!out.reserve(length) || serializeJson(doc, out) != length) {
            req->send(503, "text/plain", "Diagnostic buffer unavailable");
            return;
        }
        req->send(200, "application/json", out);
    });
#endif

#ifdef HW_MOTOR_PASSIVE_BRAKE_TEST
    server.on("/passive-brake-test", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasHeader("X-Ravlight-Brake-Test")) {
            req->send(400, "text/plain", "Use the Passive Brake Test control");
            return;
        }
        if (!orionStartPassiveBrakeTest()) {
            req->send(409, "text/plain", "Driver unavailable, DMX armed, or a brake test is already active");
            return;
        }
        req->send(202, "text/plain", "Passive brake test requested");
    });
    server.on("/passive-brake-stop", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!orionStopPassiveBrakeTest()) {
            req->send(409, "text/plain", "No active brake test");
            return;
        }
        req->send(202, "text/plain", "Holding current restore requested");
    });
#endif

    // POST /home — trigger sensorless homing
    server.on("/home", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        // Manual mode owns homing — automatic sensorless would immediately
        // trip on load. Force the user to use /sethome instead.
        if (orionConfig.manualMode) {
            req->send(409, "text/plain",
                      "manual mode active — use /sethome after jogging to home position");
            return;
        }
        drv->startHoming();
        ESP_LOGI(TAG, "homing triggered via /home");
        req->send(200, "text/plain", "homing started");
    });

    // GET /api/motor-limits — expose backend hard caps so the JS UI can
    // mirror them in the input `max` attributes. Values are in step-based
    // units (steps/s, steps/s²); the JS converts to display units (cm/s).
    server.on("/api/motor-limits", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<128> doc;
        doc["maxSpeedSteps"] = (uint32_t)ORION_MAX_SPEED_STEPS;
        doc["maxAccelSteps"] = (uint32_t)ORION_MAX_ACCEL_STEPS;
        doc["maxJogSteps"]   = (uint32_t)ORION_MAX_JOG_STEPS;
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // GET /sglog — dump the driver's SG diagnostic ring buffer (~25 s of
    // 100 ms samples, newest first). Fields per entry: t (ms), sg, spd
    // (steps/s), st (MotorState), dg (DIAG pin), gr (in grace), ig
    // (ignore-stall). Lets a block-test be analysed via curl without a
    // serial cable.
    server.on("/sglog", HTTP_GET, [](AsyncWebServerRequest* req) {
        Tmc2209LocalDriver* tmc = orionGetTmcDriver();
        if (!tmc) { req->send(503, "text/plain", "driver unavailable"); return; }
        uint64_t before = 0;
        uint64_t limit = req->hasParam("before") ? 32 : Tmc2209LocalDriver::SGLOG_N;
        if (!parseSgLogParam(req, "before", before) || !parseSgLogParam(req, "limit", limit) ||
            (req->hasParam("limit") && limit > 64)) {
            req->send(400, "text/plain", "Use limit=1..64 and the X-Next-Before cursor");
            return;
        }
        auto entries = std::unique_ptr<SgLogResponse::Entry[]>(new (std::nothrow) SgLogResponse::Entry[limit]);
        if (!entries) { req->send(503, "text/plain", "StallGuard snapshot unavailable"); return; }
        uint16_t count = 0;
        uint64_t next = 0;
        if (!tmc->sgLogSnapshot(entries.get(), static_cast<uint16_t>(limit), before, count, next)) {
            req->send(410, "text/plain", "StallGuard cursor expired; restart without before");
            return;
        }
        auto* response = new (std::nothrow) SgLogResponse(std::move(entries), count);
        if (!response) { req->send(503, "text/plain", "StallGuard response unavailable"); return; }
        response->addHeader("Cache-Control", "no-store");
        if (next) {
            char cursor[21];
            snprintf(cursor, sizeof(cursor), "%llu", static_cast<unsigned long long>(next));
            response->addHeader("X-Next-Before", cursor);
        }
        req->send(response);
    });

    // GET /sglive — live StallGuard readout + motor state for the wizard's
    // Homing step. Polled at ~200 ms while the user jogs the motor to see
    // the free-run SG value; homing SGTHRS should sit ~15-20 below that.
    server.on("/sglive", HTTP_GET, [](AsyncWebServerRequest* req) {
        IMotorDriver* drv = orionGetDriver();
        StaticJsonDocument<192> doc;
        if (!drv) {
            doc["available"] = false;
        } else {
            MotorStatus s = drv->getStatus();
            doc["available"] = true;
            doc["sg"]        = s.sg_result;
            doc["state"]     = stateName(s.state);
            doc["homed"]     = s.homed;
            doc["pos"]       = s.position;
            doc["operSgthrs"] = orionConfig.operSgthrs;
        }
        String out;
        serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // POST /sethome — manual override: zero the position counter and declare
    // homed at the current physical position. Use when sensorless homing isn't
    // available (driver without current sensing) or when "home" is a chosen
    // operating position rather than a mechanical end stop.
    server.on("/sethome", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        drv->setHomePosition();
        ESP_LOGI(TAG, "home set manually via /sethome");
        req->send(200, "text/plain", "home set");
    });

    // POST /manualmode?on=1|0 — toggle the persistent manual-override mode.
    // In manual mode every automatic safety feature is disabled (stall,
    // watchdog, auto-rehome) and /home refuses to run. Persists to NVS
    // and immediately re-pushes the homing config so SG threshold flips
    // to 0 (or back to operSgthrs) without waiting for a reboot.
    server.on("/manualmode", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        if (!req->hasParam("on", true)) {
            req->send(400, "text/plain", "missing 'on' param (1 or 0)");
            return;
        }
        bool on = req->getParam("on", true)->value() == "1";
        orionConfig.manualMode = on;
        orionApplyHomingConfig();
        saveConfig();
        ESP_LOGW(TAG, "manual mode %s (persisted)", on ? "ENABLED" : "disabled");
        req->send(200, "application/json",
                  on ? "{\"manualMode\":true}" : "{\"manualMode\":false}");
    });

    // POST /sgcal?dir=up|down — start a StallGuard calibration run. Motor turns at
    // homing speed in the chosen direction (default = homing direction), stall fault
    // suppressed so the operator can load the shaft and read sg. Gated while DMX armed.
    server.on("/sgcal", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        if (dmxIsActive() && orionDmxLastEnable() != 0) {
            req->send(409, "text/plain", "DMX is armed — set Enable to 0 on the console first");
            return;
        }
        int8_t d = orionConfig.homingDirection;
        if (req->hasParam("dir", true)) {
            String s = req->getParam("dir", true)->value();
            if (s == "up") d = +1; else if (s == "down") d = -1;
        }
        drv->startStallGuardCal(d);
        ESP_LOGI(TAG, "StallGuard calibration started via /sgcal (dir=%d)", (int)d);
        req->send(200, "text/plain", "sg cal running");
    });

    // POST /sgcalstop — stop the StallGuard calibration run
    server.on("/sgcalstop", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        drv->stopStallGuardCal();
        ESP_LOGI(TAG, "StallGuard calibration stopped via /sgcalstop");
        req->send(200, "text/plain", "sg cal stopped");
    });

    // POST /sgthreshold?value=N — save the operating StallGuard threshold computed
    // by the calibration wizard (1..255). Persists to NVS and applies live.
    server.on("/sgthreshold", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        if (!req->hasParam("value", true)) { req->send(400, "text/plain", "missing value"); return; }
        int v = req->getParam("value", true)->value().toInt();
        if (v < 1)   v = 1;
        if (v > 255) v = 255;
        orionSetOperSgthrs((uint8_t)v);
        req->send(200, "text/plain", String(v));
    });

    // POST /sgsweep?dir=up|down — start adaptive SG profile sweep at the
    // user's operating speed range. Motor runs at 8 bins between base and
    // base + 7×step (step/s). Each bin samples SG for ~1.5 s. Defaults:
    // base = max(3000, maxSpeed/8), step = maxSpeed/8 — covers the
    // configured maxSpeed at the top bin.
    server.on("/sgsweep", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        if (!req->hasParam("dir", true)) {
            req->send(400, "text/plain", "missing dir"); return;
        }
        String dir = req->getParam("dir", true)->value();
        int8_t d = (dir == "up") ? +1 : (dir == "down") ? -1 : 0;
        if (d == 0) { req->send(400, "text/plain", "dir must be up or down"); return; }

        uint32_t step = req->hasParam("step", true)
            ? (uint32_t)req->getParam("step", true)->value().toInt()
            : (orionConfig.maxSpeed / ORION_SGP_BINS);
        if (step < 500) step = 500;
        uint32_t base = req->hasParam("base", true)
            ? (uint32_t)req->getParam("base", true)->value().toInt()
            : step;
        if (base < 3000) base = 3000;  // floor: StallGuard valid only above this

        if (!orionStartSgSweep(d, base, step)) {
            req->send(409, "text/plain", "sweep refused (motor not idle, not homed, or already running)");
            return;
        }
        req->send(200, "text/plain", "sweep started");
    });

    // GET /sgsweep — sweep progress for the UI wizard
    server.on("/sgsweep", HTTP_GET, [](AsyncWebServerRequest* req) {
        OrionSgSweepProgress p = orionGetSgSweepProgress();
        DynamicJsonDocument doc(256);
        doc["phase"]       = p.phase;
        doc["currentBin"]  = p.current_bin;
        doc["totalBins"]   = p.total_bins;
        doc["direction"]   = (int)p.direction;
        doc["lastSg"]      = p.last_sg;
        doc["baseSpeed"]   = p.base_speed;
        doc["speedStep"]   = p.speed_step;
        doc["capped"]      = p.capped;
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // POST /sgsweepstop — abort an in-progress sweep
    server.on("/sgsweepstop", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        orionAbortSgSweep();
        req->send(200, "text/plain", "aborted");
    });

    // POST /clearfault — recover from FAULT (and optionally trigger rehoming)
    server.on("/clearfault", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        if (!orionGetDriver()) { req->send(503, "text/plain", "driver unavailable"); return; }
        bool ok = orionClearFaultAndMaybeHome();
        ESP_LOGI(TAG, "clearfault via /clearfault: %s", ok ? "ok" : "still faulted");
        req->send(ok ? 200 : 409, "text/plain", ok ? "cleared" : "fault persists");
    });

    // POST /estop — emergency stop
    server.on("/estop", HTTP_POST, [](AsyncWebServerRequest* req) {
#ifdef HW_MOTOR_PASSIVE_BRAKE_TEST
        if (orionStopPassiveBrakeTest(true)) {
            req->send(202, "text/plain", "Brake test stop requested");
            return;
        }
#endif
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        drv->estop();
        ESP_LOGW(TAG, "e-stop via /estop");
        req->send(200, "text/plain", "estopped");
    });

    // POST /moveto?pos=N — manual position command (drag-to-jog on the web UI
    // winch diagram, and ad-hoc testing). Enters manual override like /jog so
    // a concurrent DMX frame doesn't fight the drag; operator must
    // /release-dmx (or the console cycles Enable) to hand control back.
    server.on("/moveto", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req, true)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        if (!req->hasParam("pos", true)) {
            req->send(400, "text/plain", "missing pos");
            return;
        }
        int32_t pos = req->getParam("pos", true)->value().toInt();
#ifdef RAVLIGHT_ORION_GRAVITY_RECOVERY
        orionQueueWebPosition(pos);
#else
        drv->moveTo(pos);
        orionEnterManualOverride();
#endif
        req->send(200, "text/plain", "ok");
    });

    // POST /highlight — visual identify jog
    server.on("/highlight", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        fixtureHighlight();
        req->send(200, "text/plain", "ok");
    });

    // POST /jog?dir=up|down — start continuous jog at jogSpeed (hold-to-jog UX).
    // "up" maps to +1 direction, "down" to -1. The caller (UI) should issue
    // /jogstop on button release. Bypasses soft limits — calibration only.
    //
    // Safety gate: refused while DMX is actively armed (Enable != 0). Operator
    // must disarm the console first to avoid fighting between DMX target and jog.
    server.on("/jog", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        // No DMX-armed gate: jog takes manual control immediately and DMX
        // position commands are suppressed via orionEnterManualOverride()
        // until either the operator clicks Release to DMX or the console
        // cycles Enable 0→1 (see applyEnable in dmx_fixture.cpp).
        if (!req->hasParam("dir", true)) {
            req->send(400, "text/plain", "missing dir"); return;
        }
        String dir = req->getParam("dir", true)->value();
        int8_t d = (dir == "up") ? +1 : (dir == "down") ? -1 : 0;
        if (d == 0) { req->send(400, "text/plain", "dir must be up or down"); return; }
        // Before homing, clamp jog to a safe slow speed: the position counter
        // is meaningless and the operator is typically searching for the end
        // stop or limit positions. After homing, use the user-configured speed.
        // Optional per-request 'speed' param lets the wizard drop the rate
        // for fine positioning during Travel-limits step without touching
        // the persisted jogSpeed.
        float jogSpeed = (float)orionConfig.jogSpeed;
        if (req->hasParam("speed", true)) {
            float sp = req->getParam("speed", true)->value().toFloat();
            if (sp > 10.0f) jogSpeed = sp;  // ignore obvious junk
        }
        // Hard cap — the HTML max attribute only warns, it doesn't block
        // the value from being read and sent. An over-limit speed reaches
        // FastAccelStepper as-is and the motor silently loses steps.
        if (jogSpeed > (float)ORION_MAX_JOG_STEPS) jogSpeed = (float)ORION_MAX_JOG_STEPS;
        if (!drv->getStatus().homed && jogSpeed > 1500.0f) jogSpeed = 1500.0f;
        // Optional override (set from the manual-jog "Override" toggle in
        // the UI or implicitly by the setup wizard's Travel-limits step).
        // Suspends BOTH soft limits AND stall detection so the operator can
        // drive past saved limits to define new ones, without a static
        // SGTHRS that isn't tuned for this rig false-tripping. Cleared on
        // /jogstop.
        bool override_on = req->hasParam("override", true) &&
                           req->getParam("override", true)->value() == "1";
        orionSetJogIgnoreLimits(override_on);
        orionSetJogIgnoreStall (override_on);
        drv->jog(d, jogSpeed);
        orionEnterManualOverride();   // suppress DMX motion until /release-dmx
        req->send(200, "text/plain", "jogging");
    });

    // POST /jogstop — end the current jog burst. Uses hardStop (immediate
    // step pulse abort) instead of stop (decelerated ramp) so the motor
    // reacts within one tick of the button release. A decelerated stop
    // was leaving the motor coasting for hundreds of ms past release,
    // occasionally overshooting physical limits at high jog speed.
    //
    // The override flags (jog ignore-limits, jog ignore-stall) stay on
    // through the residual coast and until /release-dmx or the next
    // fresh /jog call — clearing them here would still false-trip a
    // stall while the motor decays kinetically.
    server.on("/jogstop", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        drv->hardStop();
        req->send(200, "text/plain", "stopped");
    });

    // POST /release-dmx — exit manual override; DMX position commands resume.
    server.on("/release-dmx", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        orionReleaseToDmx();
        req->send(200, "text/plain", "released");
    });

    // POST /setlimit?which=down|up — capture current position as soft limit.
    // Persists config to NVS. Same operation that DMX function bytes 150-199/200-255 trigger.
    // Safety gate: only blocked if DMX is armed AND we're NOT in manual override
    // (the operator may have jogged here intentionally to set the limit).
    server.on("/setlimit", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (rejectDuringPassiveBrakeTest(req)) return;
        IMotorDriver* drv = orionGetDriver();
        if (!drv) { req->send(503, "text/plain", "driver unavailable"); return; }
        if (dmxIsActive() && orionDmxLastEnable() != 0 && !orionManualOverride()) {
            req->send(409, "text/plain", "DMX is armed — jog to override or set Enable to 0 first");
            return;
        }
        if (!req->hasParam("which", true)) {
            req->send(400, "text/plain", "missing which"); return;
        }
        String which = req->getParam("which", true)->value();
        int32_t pos = drv->getStatus().position;
        if (which == "down") {
            orionConfig.downPosition = pos;
        } else if (which == "up") {
            orionConfig.upPosition = pos;
        } else {
            req->send(400, "text/plain", "which must be up or down"); return;
        }
        orionApplySoftLimitsExternal();
        saveConfig();
        float cm = (float)pos / orionStepsPerCm();
        ESP_LOGI(TAG, "set %s limit via /setlimit: %d steps (%.1f cm)", which.c_str(), pos, cm);
        req->send(200, "text/plain", String(cm, 1));
    });

#ifdef ORION_HAS_LED
    // POST /ledhighlight?out=i — white-wipe identification on one LED output
    server.on("/ledhighlight", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("out", true)) { req->send(400, "text/plain", "missing out"); return; }
        orionHighlightLed(req->getParam("out", true)->value().toInt());
        req->send(200, "text/plain", "ok");
    });
#endif
}

#endif // RAVLIGHT_FIXTURE_ORION
