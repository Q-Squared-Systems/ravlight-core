#ifdef RAVLIGHT_HAS_MOTOR
#include "core/motor/backends/Tmc2209LocalDriver.h"
#include "fixtures/orion/motor_event_log.h"
#include <Arduino.h>
#include <string.h>
#include <esp_log.h>
#include <esp_system.h>
#ifdef HW_MOTOR_RETAIN_ENABLE
#include <driver/gpio.h>
#include <soc/rtc_cntl_reg.h>
#endif

static const char* TAG = "TMC2209Local";

// Periodic register read interval (ms)
static constexpr uint32_t REG_READ_INTERVAL_MS = 100;

// Short blank window — covers just the acceleration ramp before SG sampling
// kicks in. The settle-then-watch state machine below handles PWM autoscale.
static constexpr uint32_t HOMING_STALL_BLANK_MS  = 300;

// Consecutive SG samples above trip needed to declare "settled" (free-run
// reached, autoscale converged). At 100 ms sample rate, 10 = 1000 ms of stable
// high SG. Covers PWM autoscale re-tune after a prior stall event so a
// second consecutive homing doesn't false-trip on the transient.
static constexpr uint8_t  HOMING_SG_SETTLE_HIGH  = 10;

// Maximum time we wait for SG to first cross above the trip threshold. If the
// motor is jammed against an end stop from the start, SG never rises, so this
// is the upper bound on how long the motor pushes before we declare stall.
static constexpr uint32_t HOMING_SETTLE_TIMEOUT_MS = 2500;

// Consecutive SG samples below trip required to declare a stall once settled.
// At 100 ms sample rate this is 500 ms of sustained low SG — filters out
// natural SG oscillation (±10-15 points around free-run regime).
static constexpr uint8_t  HOMING_SG_DEBOUNCE     = 5;

// EN pin polarity: TMC2209 EN is active LOW
static constexpr uint8_t EN_ENABLE  = LOW;
static constexpr uint8_t EN_DISABLE = HIGH;

// ─── Constructor / Destructor ────────────────────────────────────────────────

Tmc2209LocalDriver::Tmc2209LocalDriver(HardwareSerial& serial,
                                       uint8_t  tmc_address,
                                       uint8_t  step_pin,
                                       uint8_t  dir_pin,
                                       uint8_t  en_pin,
                                       uint8_t  rx_pin,
                                       uint8_t  tx_pin,
                                       uint8_t  diag_pin,
                                       float    r_sense,
                                       uint32_t uart_baud)
    : _serial(serial),
      _tmc_addr(tmc_address),
      _step_pin(step_pin), _dir_pin(dir_pin), _en_pin(en_pin),
      _rx_pin(rx_pin), _tx_pin(tx_pin), _diag_pin(diag_pin),
      _r_sense(r_sense), _uart_baud(uart_baud)
{}

Tmc2209LocalDriver::~Tmc2209LocalDriver() {
    end();
}

// ─── Lifecycle ───────────────────────────────────────────────────────────────

bool Tmc2209LocalDriver::begin() {
    _sglog_boot = esp_random();
#ifdef HW_MOTOR_RETAIN_ENABLE
    // GPIO32 is the ESP32 X32P RTC pad. Read its hardware latch before
    // modifying the pin. This feature is enabled only on the v6 board.
    static_assert(HW_PIN_MOTOR_EN == 32, "Update the ENN hold register for this pin");
    _retained_enable_at_boot = REG_GET_BIT(RTC_CNTL_HOLD_FORCE_REG, RTC_CNTL_X32P_HOLD_FORCE);
    digitalWrite(_en_pin, _retained_enable_at_boot ? EN_ENABLE : EN_DISABLE);
    pinMode(_en_pin, OUTPUT);
#else
    // Configure output pins — keep driver disabled until fully initialised
    pinMode(_en_pin,   OUTPUT); digitalWrite(_en_pin,   EN_DISABLE);
#endif
    pinMode(_step_pin, OUTPUT); digitalWrite(_step_pin, LOW);
    pinMode(_dir_pin,  OUTPUT); digitalWrite(_dir_pin,  LOW);
    pinMode(_diag_pin, INPUT);

    // UART for TMCStepper — half-duplex PDN_UART
    _serial.begin(_uart_baud, SERIAL_8N1, _rx_pin, _tx_pin);
    delay(50);

    // TMC2209 via TMCStepper
    _tmc = new TMC2209Stepper(&_serial, _r_sense, _tmc_addr);
#ifdef HW_MOTOR_RETAIN_ENABLE
    bool driver_still_enabled = false;
    if (_retained_enable_at_boot) {
        TMC2209_n::IOIN_t pins{};
        pins.sr = _tmc->IOIN();
        const uint32_t chopper = _tmc->CHOPCONF();
        const auto status = _tmc->GSTAT();
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
        _observeDriverStatus(status, !_tmc->CRCerror, true);
#endif
        driver_still_enabled = pins.version == 0x21 && !pins.enn &&
                               (chopper & 0x0F) != 0 && !_tmc->CRCerror && status == 0;
#ifdef HW_MOTOR_PASSIVE_BRAKE_TEST
        if (driver_still_enabled) {
            // A reset can interrupt the test with the driver still powered
            // but in passive brake mode. Restore the normal startup config.
            const uint8_t freewheel = _tmc->freewheel();
            driver_still_enabled = !_tmc->CRCerror && freewheel == 0;
        }
#endif
        if (driver_still_enabled) {
            // Seed TMCStepper's register cache for later current changes.
            // Preserve the driver's live chopper value and hold current.
            _tmc->CHOPCONF(chopper);
        }
    }
    if (!driver_still_enabled) {
        _setDriverEnabled(false);
        _configureTmc();
    }
#else
    _configureTmc();
#endif

    // Verify UART communication
    if (_tmc->test_connection() != 0) {
        ESP_LOGE(TAG, "TMC2209 UART not responding (addr=%d)", _tmc_addr);
        return false;
    }
    ESP_LOGI(TAG, "TMC2209 addr=%d ok, version=0x%02X", _tmc_addr, _tmc->version());
#ifdef HW_MOTOR_RETAIN_ENABLE
    if (!driver_still_enabled)
        _tmc->GSTAT(0x07);  // Acknowledge initial status for the next retained boot.
#endif

    // FastAccelStepper — step pulses generated by ESP32 hardware (MCPWM/RMT).
    _engine.init();
    _stepper = _engine.stepperConnectToPin(_step_pin);
    if (!_stepper) {
        ESP_LOGE(TAG, "FastAccelStepper: no step-gen hardware free for GPIO%d", _step_pin);
        return false;
    }
    _stepper->setDirectionPin(_dir_pin);
    _stepper->setSpeedInHz(2000);     // overridden by setSpeed()
    _stepper->setAcceleration(1000);  // overridden by setAccel()

    // DIAG interrupt — RISING edge when TMC2209 detects stall
    attachInterruptArg(digitalPinToInterrupt(_diag_pin), _diagIsrCb, this, RISING);

#ifdef RAVLIGHT_TMC_STEALTHCHOP_TRIAL
    // Apply to both cold starts and retained starts while STEP is idle.
    _configureStealthChopTrial();
#endif
    _setDriverEnabled(true);
    _state       = MotorState::IDLE;
    _setFaultFlags(0);
    _homed       = false;

    return true;
}

void Tmc2209LocalDriver::_configureTmc() {
    _tmc->begin();
    _tmc->toff(4);                      // chopper off-time: enables driver output stage
    _tmc->blank_time(36);               // longer TBL → smoother chopper, less hiss
    _tmc->intpol(true);                 // 256-microstep interpolation: smoother current
    // Set both IRUN and IHOLD explicitly. The single-arg rms_current() leaves
    // IHOLD at the library default (~50% of IRUN), which kept the motor at
    // ~400 mA at idle and caused audible chopper hum even with pwm_freq=3.
    {
        float hold_mult = (_run_ma > 0) ? (float)_hold_ma / _run_ma : 0.0f;
        _tmc->rms_current(_run_ma, hold_mult);
    }
    _tmc->microsteps(16);               // 1/16 microstepping
    // SpreadCycle from boot — silent on this motor in motion AND at idle.
    // The StealthChop hum is motor-specific and we never reach a silent
    // chopper state with it on this hardware.
#ifndef RAVLIGHT_TMC_STEALTHCHOP_TRIAL
    _tmc->en_spreadCycle(true);
#endif
    // SGTHRS = 0 means DIAG never fires from stall comparison — set higher
    // only when entering homing/SGCal. Prevents spurious stall faults on jog.
    _tmc->SGTHRS(0);
    // PWM chopper frequency: 0=~23 kHz, 1=~35 kHz (datasheet default),
    // 2=~47 kHz, 3=~58 kHz. Higher freq is "more ultrasonic" in theory but
    // can hit motor acoustic resonance modes. 1 is the manufacturer-recommended
    // default — try others if this resonates on the specific motor.
    _tmc->pwm_freq(2);
    // Keep coils energised at IHOLD on standstill so the winch has holding
    // torque from boot — freewheel was tried (FREEWHEEL=1) but produced an
    // audible low-pitched hum on this hardware and made the rope freely
    // releasable at idle.
    _tmc->freewheel(0);
    // Short TPOWERDOWN so current drops to IHOLD ~310 ms after motion ends —
    // default ~3 s leaves the motor at full IRUN heating up after each move.
    _tmc->TPOWERDOWN(2);
    _tmc->TCOOLTHRS(0xFFFFF);           // enable CoolStep / StallGuard at all speeds
    // CoolStep: monitor SG_RESULT and reduce drive current when load is light.
    // semin=5 → ramp current up when SG_RESULT drops below 32*(semin+1) = 192.
    // semax=2 → ramp current down when SG_RESULT > 32*(semin+semax+1) = 256.
    // sedn=01 → current decrement rate (1 step per 32 SG reads).
    // seimin=1 → minimum CoolStep-reduced current = IRUN/2 (i.e. ~250 mA).
    // Keeps the motor cool during long DMX sinusoid sessions where the load
    // is typically much lower than the configured peak run current.
    _tmc->semin(5);
    _tmc->semax(2);
    _tmc->sedn(0b01);
    _tmc->seimin(1);
}

#if defined(RAVLIGHT_TMC_STEALTHCHOP_TRIAL) || defined(HW_MOTOR_PASSIVE_BRAKE_TEST)
// GCONF writes take the raw setting. Readback exposes the effective value
// after the SPREAD pin inverts bit 2, so the conversion works both ways.
static constexpr uint32_t invertGconfSpreadBit(uint32_t gconf, bool spreadPinHigh) {
    return gconf ^ (spreadPinHigh ? (1u << 2) : 0u);
}

static_assert(invertGconfSpreadBit(0x1C1, true) == 0x1C5, "Restore StealthChop with SPREAD high");
static_assert(invertGconfSpreadBit(0x1C5, true) == 0x1C1, "Restore SpreadCycle with SPREAD high");
static_assert(invertGconfSpreadBit(0x1C1, false) == 0x1C1, "Preserve GCONF with SPREAD low");
#endif

#ifdef RAVLIGHT_TMC_STEALTHCHOP_TRIAL
void Tmc2209LocalDriver::_configureStealthChopTrial() {
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
    ProfileTrace trace;
    trace.attempts = 1;
    trace.expectedWrites = 5;
    const auto recordFailure = [&](const char* reason) {
        trace.result = reason;
        trace.readback = _captureProfileRegisters();
        _recordProfileTrace(false, trace);
    };
#endif

    TMC2209_n::IOIN_t pins{};
    pins.sr = _tmc->IOIN();
    if (_tmc->CRCerror) {
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
        recordFailure("ioin_read_failed");
#endif
        return;
    }

    TMC2208_n::GCONF_t gconf{};
    gconf.sr = _tmc->GCONF();
    if (_tmc->CRCerror) {
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
        recordFailure("gconf_read_failed");
#endif
        return;
    }

    TMC2208_n::PWMCONF_t pwmconf{};
    pwmconf.sr = _tmc->PWMCONF();
    if (_tmc->CRCerror) {
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
        recordFailure("pwmconf_read_failed");
#endif
        return;
    }

#ifdef RAVLIGHT_TMC_COOLSTEP_TRIAL
    const uint8_t initialWriteCount = _tmc->IFCNT();
    if (_tmc->CRCerror) {
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
        trace.ifcntBefore = {initialWriteCount, false};
        recordFailure("ifcnt_before_read_failed");
#endif
        return;
    }

    // TSTEP counts clock ticks at the equivalent 1/256 microstep rate.
    // 250 ticks is nominally 3,000 input steps/s at 1/16 and a 12 MHz clock.
    _tmc->TCOOLTHRS(250);
    TMC2209_n::COOLCONF_t coolconf{};
    coolconf.semin = 10; // Restore current below SG_RESULT 320.
    coolconf.semax = 2;  // Reduce current only above SG_RESULT 416.
    coolconf.seup = 3;   // Raise the current scale by eight per increment.
    coolconf.sedn = 0;   // Wait 32 measurements per decrement.
    coolconf.seimin = 0; // Select the half-IRUN scale floor.
    _tmc->COOLCONF(coolconf.sr);
#else
    // Keep current fixed in the StealthChop comparison build.
    _tmc->semin(0);
#endif
    _tmc->TPWMTHRS(0); // No automatic switch to SpreadCycle at speed.
    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_autograd = true;
    _tmc->PWMCONF(pwmconf.sr);
    gconf.en_spreadcycle = pins.spread_en; // The SPREAD pin XORs this bit.
    _tmc->GCONF(gconf.sr);
#ifdef RAVLIGHT_TMC_COOLSTEP_TRIAL
    const uint8_t finalWriteCount = _tmc->IFCNT();
    const bool counterValid = !_tmc->CRCerror;
    _coolStepConfigured.store(counterValid && finalWriteCount == static_cast<uint8_t>(initialWriteCount + 5),
                              std::memory_order_relaxed);
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
    trace.targetKnown = true;
    trace.targetGconf = gconf.sr;
    trace.expectedEffectiveGconf = invertGconfSpreadBit(gconf.sr, pins.spread_en);
    trace.targetPwmconf = pwmconf.sr;
    trace.ifcntBefore = {initialWriteCount, true};
    trace.ifcntAfter = {finalWriteCount, counterValid};
    trace.result = !counterValid ? "ifcnt_after_read_failed" :
                   coolStepConfigured() ? "writes_acknowledged" : "ifcnt_delta_mismatch";
    trace.readback = _captureProfileRegisters();
    if (coolStepConfigured()) {
        if (!trace.readback.gconf.valid) trace.result = "gconf_readback_failed";
        else if (trace.readback.gconf.value != trace.expectedEffectiveGconf) trace.result = "gconf_mismatch";
        else if (!trace.readback.pwmconf.valid) trace.result = "pwmconf_readback_failed";
        else if (trace.readback.pwmconf.value != trace.targetPwmconf) trace.result = "pwmconf_mismatch";
        else trace.result = "register_checks_passed";
    }
    _recordProfileTrace(false, trace);
#endif
#endif
}

const char* Tmc2209LocalDriver::chopperModeAtIdle() const {
    switch (_idleChopperMode.load(std::memory_order_relaxed)) {
        case ChopperMode::STEALTH_CHOP: return "StealthChop";
        case ChopperMode::SPREAD_CYCLE: return "SpreadCycle";
        default: return "unknown";
    }
}
#endif

void Tmc2209LocalDriver::_setDriverEnabled(bool enabled) {
    digitalWrite(_en_pin, enabled ? EN_ENABLE : EN_DISABLE);
#ifdef HW_MOTOR_RETAIN_ENABLE
    const auto pin = static_cast<gpio_num_t>(_en_pin);
    if (enabled) {
        gpio_hold_en(pin);
    } else {
        // Preload HIGH before releasing the latch on an intentional disable.
        gpio_hold_dis(pin);
    }
#endif
}

void Tmc2209LocalDriver::end() {
    if (_diag_pin < 255)
        detachInterrupt(digitalPinToInterrupt(_diag_pin));

    if (_stepper) _stepper->forceStop();
    _setDriverEnabled(false);
    _state = MotorState::DRIVER_OFF;

    // FastAccelStepper instances are owned by the engine — not deleted here.
    _stepper = nullptr;
    delete _tmc; _tmc = nullptr;
}

// ─── Motion ──────────────────────────────────────────────────────────────────

void Tmc2209LocalDriver::moveTo(int32_t position) {
    if (_state == MotorState::FAULT || _state == MotorState::DRIVER_OFF) return;

    // Apply soft limits — clamp target into [min, max]
    if (_soft_limits_enabled) {
        if (position < _soft_min) {
            ESP_LOGW(TAG, "moveTo %d clamped to soft_min %d", position, _soft_min);
            position = _soft_min;
        } else if (position > _soft_max) {
            ESP_LOGW(TAG, "moveTo %d clamped to soft_max %d", position, _soft_max);
            position = _soft_max;
        }
    }

    // Motion-transient grace for moves that start from standstill (SG is
    // invalid until the motor spins up — see jog() for the full rationale).
    // Deliberately NOT re-armed while already moving: DMX tracking calls
    // moveTo() every frame and a rolling grace would permanently disable
    // stall detection during DMX-driven motion.
    if (_stepper->getCurrentSpeedInMilliHz() == 0 &&
        position != _stepper->getCurrentPosition())
        _stop_grace_until_ms = millis() + 800;

    _stepper->moveTo(position);
    if (position != _stepper->getCurrentPosition())
        _state = MotorState::MOVING;
}

void Tmc2209LocalDriver::moveBy(int32_t delta) {
    moveTo(_stepper->getCurrentPosition() + delta);
}

void Tmc2209LocalDriver::setSpeed(float steps_per_sec) {
    // Sanity cap independent of fixture-level limits — TMC2209 step
    // generation is unreliable beyond ~100 kHz on the ESP32 timing path,
    // and physical stalls become certain long before that. Fixture layer
    // already clamps against its own tighter caps; this is defense in depth.
    constexpr uint32_t DRV_MAX_HZ = 100000;
    uint32_t hz = (steps_per_sec > 1.0f) ? (uint32_t)steps_per_sec : 1;
    if (hz > DRV_MAX_HZ) hz = DRV_MAX_HZ;
    _stepper->setSpeedInHz(hz);
}

void Tmc2209LocalDriver::setAccel(float steps_per_sec2) {
    // Same reasoning as setSpeed — driver-level cap is a soft ceiling
    // (matched to TMC2209 realistic envelope); fixture layer enforces
    // application-specific limits.
    constexpr uint32_t DRV_MAX_HZ2 = 200000;
    uint32_t a = (steps_per_sec2 > 1.0f) ? (uint32_t)steps_per_sec2 : 1;
    if (a > DRV_MAX_HZ2) a = DRV_MAX_HZ2;
    _stepper->setAcceleration(a);
}

void Tmc2209LocalDriver::stop() {
    // Snapshot current motion BEFORE issuing stopMove — after stopMove the
    // ramp generator starts winding down and speed already begins to fall.
    int32_t spd_mhz_now = _stepper->getCurrentSpeedInMilliHz();
    uint32_t spd_hz     = (uint32_t)(abs(spd_mhz_now) / 1000);
    uint32_t accel_hz2  = _stepper->getAcceleration();
    if (accel_hz2 == 0) accel_hz2 = 1000;  // defensive

    _stepper->stopMove();
    if (_state == MotorState::JOGGING)
        _state = MotorState::IDLE;

    // Arm the post-stop grace window dynamically. During the deceleration
    // ramp SG_RESULT drops linearly with speed, so a decelerating motor
    // looks exactly like a stalling one to hardware-level StallGuard4.
    // Grace must cover the entire ramp, otherwise DIAG can fire *after*
    // the window closes but *before* speed reaches zero — the pattern
    // reported at high jog speed.
    //
    //   ramp_ms ≈ (speed / accel) × 1000, with 60% safety margin
    //   clamped to [800, 4000] ms to bound the window on both sides.
    uint32_t ramp_ms = ((uint64_t)spd_hz * 1000UL * 16UL / (10UL * accel_hz2));
    if (ramp_ms <  800) ramp_ms = 800;
    if (ramp_ms > 4000) ramp_ms = 4000;
    _stop_grace_until_ms = millis() + ramp_ms;
}

void Tmc2209LocalDriver::jog(int8_t direction, float speed_sps) {
    if (_state == MotorState::FAULT || _state == MotorState::DRIVER_OFF) {
        ESP_LOGW(TAG, "jog rejected: driver not ready (state=%d)", (int)_state);
        return;
    }
    if (direction == 0 || speed_sps <= 0.0f) {
        stop();
        return;
    }
    // Same physical ceiling as setSpeed() — jog used to write setSpeedInHz
    // directly, bypassing the clamp, so an out-of-range request spun the
    // step generator faster than the motor can follow (silent lost steps).
    constexpr uint32_t DRV_MAX_HZ = 100000;
    uint32_t hz = (speed_sps > 1.0f) ? (uint32_t)speed_sps : 1;
    if (hz > DRV_MAX_HZ) hz = DRV_MAX_HZ;

    // Motion-transient grace: SG_RESULT is garbage at standstill (reads
    // 10-60 on a motor that free-runs at 250-350) and takes a few hundred
    // ms to climb once motion starts. A jog from standstill — or a hard
    // direction reversal — therefore looks exactly like a stall to both
    // the DIAG edge and the level fallback for the first readings. Arm
    // the same grace window used for deceleration so all stall paths hold
    // off until the transient settles.
    int32_t cur_mhz = _stepper->getCurrentSpeedInMilliHz();
    bool from_standstill = (cur_mhz == 0);
    bool dir_reversal    = (cur_mhz > 0 && direction < 0) ||
                           (cur_mhz < 0 && direction > 0);
    if (from_standstill || dir_reversal)
        _stop_grace_until_ms = millis() + 800;

    _stepper->setSpeedInHz(hz);
    if (direction > 0) _stepper->runForward();
    else               _stepper->runBackward();
    _state = MotorState::JOGGING;
}

// Internal helper — near-instant stop via accel-boost + stopMove. Preferred
// over _stepper->forceStop() because forceStop sets ignore_commands=true on
// the step queue, which then silently blocks all subsequent runForward /
// moveTo calls until either a specific reset sequence runs or the ESP is
// rebooted. The accel-boost pattern gives an equivalent physical stop
// (couple of ms decel) without the ignore_commands side effect.
void Tmc2209LocalDriver::_instantStop() {
    if (!_stepper) return;
    if (!_accel_override_active) {
        _saved_accel = _stepper->getAcceleration();
        if (_saved_accel == 0) _saved_accel = 1000;
        _stepper->setAcceleration(1000000);   // 1 M steps/s² ≈ instant
        _accel_override_active = true;
    }
    _stepper->stopMove();
    _stop_grace_until_ms = millis() + 500;
}

void Tmc2209LocalDriver::triggerStall() {
    if (!_stepper) return;
    _instantStop();   // was forceStop() — same physical result, no queue trap
    _setFaultFlags(_fault_flags | (uint8_t)MotorFault::STALL);
    _state = MotorState::FAULT;
    ESP_LOGW(TAG, "stall triggered by fixture (profile-based)");
}

void Tmc2209LocalDriver::hardStop() {
    // Near-instant stop for responsive jog release. See _instantStop for
    // why we don't use FastAccelStepper::forceStop directly.
    _instantStop();
    if (_state == MotorState::JOGGING || _state == MotorState::MOVING)
        _state = MotorState::IDLE;
}

void Tmc2209LocalDriver::estop() {
    // Near-instant stop with holding torque preserved. EN stays LOW so
    // IHOLD keeps the shaft locked — critical for winch/lift where the
    // load would drop free-fall the moment the driver was cut. The
    // operator can still de-energise the motor explicitly via driverOff()
    // (dedicated method), used e.g. by the drop-and-rehome recovery.
    _instantStop();
    _state        = MotorState::FAULT;
    _setFaultFlags(_fault_flags | (uint8_t)MotorFault::DRIVER_ERROR);
    ESP_LOGW(TAG, "E-STOP triggered (holding torque preserved)");
}

void Tmc2209LocalDriver::driverOff() {
    // Explicit de-energise. Used by the drop-and-rehome recovery flow and
    // by any operator UI wanting to release the shaft (e.g. for manual
    // re-positioning). Motor is free to move under gravity or by hand.
    if (_en_pin == 0xFF) return;
    _setDriverEnabled(false);
    _state = MotorState::DRIVER_OFF;
    ESP_LOGI(TAG, "driver disabled — shaft is free");
}

void Tmc2209LocalDriver::driverOn() {
    // Re-energise the driver after a driverOff(). Restores holding torque
    // via IHOLD but does NOT restart any motion — the position counter
    // and _homed flag are preserved (caller decides if a rehome is needed).
    if (_en_pin == 0xFF) return;
    _setDriverEnabled(true);
    if (_state == MotorState::DRIVER_OFF) _state = MotorState::IDLE;
    ESP_LOGI(TAG, "driver enabled");
}

// ─── Soft limits + fault recovery ────────────────────────────────────────────

void Tmc2209LocalDriver::setSoftLimits(int32_t min_pos, int32_t max_pos) {
    if (min_pos >= max_pos) {
        ESP_LOGW(TAG, "setSoftLimits ignored: min (%d) >= max (%d)", min_pos, max_pos);
        return;
    }
    _soft_min            = min_pos;
    _soft_max            = max_pos;
    _soft_limits_enabled = true;
    ESP_LOGI(TAG, "soft limits enabled: [%d, %d]", _soft_min, _soft_max);
}

void Tmc2209LocalDriver::disableSoftLimits() {
    _soft_limits_enabled = false;
    ESP_LOGI(TAG, "soft limits disabled");
}

bool Tmc2209LocalDriver::clearFault() {
    if (_state != MotorState::FAULT && _state != MotorState::DRIVER_OFF) {
        // Nothing to clear — but still reset flags in case they accumulated
        _setFaultFlags(0);
        return true;
    }
    if (!_tmc) {
        ESP_LOGE(TAG, "clearFault: driver not initialised");
        return false;
    }

    // Re-read driver registers to check whether sticky faults still active
    _readRegisters();

    // Sticky conditions: overtemperature must clear, driver must respond on UART
    if (_fault_flags & (uint8_t)MotorFault::OVERTEMP) {
        ESP_LOGW(TAG, "clearFault: overtemperature still active — refusing to clear");
        return false;
    }
    if (_tmc->test_connection() != 0) {
        ESP_LOGW(TAG, "clearFault: UART not responding — refusing to clear");
        _setFaultFlags(_fault_flags | (uint8_t)MotorFault::DRIVER_ERROR);
        return false;
    }

    // If the fault we're clearing was a STALL, the step counter can't
    // be trusted anymore — the motor physically slipped or the load
    // dropped, so the commanded position no longer matches the shaft
    // angle. Invalidate _homed so subsequent DMX position commands are
    // ignored until either the operator re-homes or the fixture layer
    // fires auto-rehome / drop-and-rehome recovery. Self-locking drives
    // (worm gear) opt out via setInvalidateHomeOnStall(false): a stalled
    // worm can't back-drive, so the position reference survives.
    bool was_stall = _invalidate_home_on_stall
                     && (_fault_flags & (uint8_t)MotorFault::STALL) != 0;

    // All sticky conditions cleared — reset flags, re-enable, return to IDLE
    _setFaultFlags(0);
    _stall_isr_flag = false;
    _setDriverEnabled(true);
    _state = MotorState::IDLE;
    if (was_stall) {
        _homed = false;
        ESP_LOGW(TAG, "fault cleared (post-STALL): position invalidated, re-home required");
    } else {
        ESP_LOGI(TAG, "fault cleared, driver re-enabled");
    }
    return true;
}

// ─── Homing ──────────────────────────────────────────────────────────────────

void Tmc2209LocalDriver::startHoming() {
    if (_state == MotorState::FAULT || _state == MotorState::DRIVER_OFF) {
        ESP_LOGW(TAG, "startHoming: driver not ready (state=%d)", (int)_state);
        return;
    }
    if (!_tmc || !_stepper) return;

    // Apply reduced current directly — do NOT update _run_ma so restore in _updateHoming works
    float hold_mult = (_hcfg.current_ma > 0)
                      ? (float)_hold_ma / _hcfg.current_ma : 0.0f;
    _tmc->rms_current(_hcfg.current_ma, hold_mult);
    // Already in SpreadCycle from boot — set SGTHRS for the homing trip.
    _tmc->SGTHRS(_hcfg.sgthrs);
    _sgthrs_active = _hcfg.sgthrs;

    // Clear any stale stall flag before we start moving
    _stall_isr_flag = false;

    // Constant-speed travel toward the end stop.
    _stepper->setSpeedInHz(_hcfg.speed > 1 ? _hcfg.speed : 1);
    if (_hcfg.direction > 0) _stepper->runForward();
    else                     _stepper->runBackward();

    _homing_start_ms = millis();
    _homing_phase    = HomingPhase::MOVING;
    _state           = MotorState::HOMING;
    _homing_settled  = false;   // wait for SG to first cross above trip threshold
    _sg_high_count   = 0;
    _sg_low_count    = 0;
    ESP_LOGI(TAG, "homing started: dir=%d speed=%u sgthrs=%u current=%u mA",
             _hcfg.direction, _hcfg.speed, _hcfg.sgthrs, _hcfg.current_ma);
}

void Tmc2209LocalDriver::setHomePosition() {
    _stepper->forceStopAndNewPosition(0);
    _homed = true;
    ESP_LOGI(TAG, "home position set (step counter reset to 0)");
}

void Tmc2209LocalDriver::setHomingConfig(const HomingConfig& cfg) {
    _hcfg = cfg;
    ESP_LOGI(TAG, "homing config: dir=%d speed=%u sgthrs=%u current=%u backoff=%u",
             cfg.direction, cfg.speed, cfg.sgthrs, cfg.current_ma, cfg.backoff_steps);
}

// ─── Current control ─────────────────────────────────────────────────────────

void Tmc2209LocalDriver::setRunCurrent(uint16_t ma) {
    _run_ma = ma;
    if (_tmc) {
        float hold_mult = (_run_ma > 0) ? (float)_hold_ma / _run_ma : 0.0f;
        _tmc->rms_current(_run_ma, hold_mult);
    }
}

void Tmc2209LocalDriver::setHoldCurrent(uint16_t ma) {
    _hold_ma = ma;
    setRunCurrent(_run_ma);  // re-applies hold multiplier
}

#ifdef HW_MOTOR_PASSIVE_BRAKE_TEST
static constexpr uint32_t BRAKE_RAMP_MS = 2000;
static constexpr uint32_t BRAKE_DWELL_MS = 3000;

#ifdef RAVLIGHT_ORION_GRAVITY_RECOVERY
bool Tmc2209LocalDriver::verifyRecoveryDriver() {
    TMC2209_n::IOIN_t pins{};
    pins.sr = _tmc->IOIN();
    if (_tmc->CRCerror || pins.version != 0x21 || pins.enn) return false;
    GSTAT_t general{};
    general.sr = _tmc->GSTAT();
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    _observeDriverStatus(general.sr, !_tmc->CRCerror);
#endif
    if (_tmc->CRCerror || general.reset || general.drv_err || general.uv_cp) return false;
    TMC2208_n::DRV_STATUS_t status{};
    status.sr = _tmc->DRV_STATUS();
    return !_tmc->CRCerror && !status.ot && !status.otpw &&
           !status.s2ga && !status.s2gb && !status.s2vsa && !status.s2vsb;
}
#endif

bool Tmc2209LocalDriver::beginPassiveBrakeTest(bool recovery) {
    if (!_tmc || !_stepper || _state != MotorState::IDLE || isBusy() ||
        _fault_flags) return false;

#ifdef RAVLIGHT_ORION_GRAVITY_RECOVERY
    if (recovery) {
        if (!verifyRecoveryDriver() || !_run_ma || !_hold_ma) return false;
        // The near-instant fault stop may have left emergency acceleration
        // active while the recovery task was waiting for STEP to go idle.
        if (_accel_override_active) {
            _stepper->setAcceleration(_saved_accel);
            _accel_override_active = false;
        }
        // A retained boot does not initialize TMCStepper's write-only current
        // shadow. Apply known saved currents before using that shadow to restore hold.
        const uint8_t before = _tmc->IFCNT();
        if (_tmc->CRCerror) return false;
        setRunCurrent(_run_ma);
        if (_tmc->IFCNT() != static_cast<uint8_t>(before + 3) || _tmc->CRCerror)
            return false;
    }
#endif
    if (!_tmc->ihold()) return false;
    _brake_for_recovery = recovery;

    TMC2209_n::IOIN_t pins{};
    pins.sr = _tmc->IOIN();
    if (_tmc->CRCerror || pins.version != 0x21 || pins.enn) return false;

    _brake_spread_pin_high = pins.spread_en;
    _brake_saved_gconf = _tmc->GCONF();
    if (_tmc->CRCerror) return false;
    _brake_saved_pwmconf = _tmc->PWMCONF();
    if (_tmc->CRCerror) return false;
    _brake_saved_currents = _tmc->IHOLD_IRUN();  // TMCStepper shadow of the write-only register

    TMC2208_n::GCONF_t gconf{};
    gconf.sr = _brake_saved_gconf;
    gconf.en_spreadcycle = pins.spread_en;  // The external SPREAD pin XORs this bit.
    TMC2208_n::PWMCONF_t pwmconf{};
    pwmconf.sr = _brake_saved_pwmconf;
    pwmconf.freewheel = 2;  // Low-side coil short once IHOLD reaches zero.

    _homed = false;  // Gravity-driven movement is invisible to the step counter.
    _tmc->PWMCONF(pwmconf.sr);
    _tmc->GCONF(gconf.sr);
    // Check the actual chopper mode and read back the brake selection before
    // reducing holding current. ENN remains low and STEP remains idle.
    const bool stealth = _tmc->stealth() && !_tmc->CRCerror;
    if (!stealth || _tmc->PWMCONF() != pwmconf.sr || _tmc->CRCerror) {
        endPassiveBrakeTest();
        return false;
    }

    _brake_test_start_ms = millis();
    return true;
}

Tmc2209LocalDriver::BrakeProgress Tmc2209LocalDriver::updatePassiveBrakeTest() {
    const uint32_t rampMs = _brake_for_recovery ? 4000 : BRAKE_RAMP_MS;
    const uint32_t dwellMs = _brake_for_recovery ? 14000 : BRAKE_DWELL_MS;
    const uint32_t elapsed = millis() - _brake_test_start_ms;
    const uint8_t initialHold = _brake_saved_currents & 0x1Fu;
    const uint8_t hold = elapsed >= rampMs ? 0 :
        initialHold - elapsed * initialHold / rampMs;
    if (hold != _tmc->ihold()) {
        const uint8_t before = _tmc->IFCNT();
        if (_tmc->CRCerror) return BrakeProgress::FAILED;
        _tmc->ihold(hold);
        if (_tmc->IFCNT() != static_cast<uint8_t>(before + 1) || _tmc->CRCerror)
            return BrakeProgress::FAILED;
        if (!hold) _brake_test_start_ms = millis() - rampMs; // Dwell starts at verified zero.
        return BrakeProgress::RUNNING;
    }
    return elapsed >= rampMs + dwellMs ? BrakeProgress::COMPLETE : BrakeProgress::RUNNING;
}

bool Tmc2209LocalDriver::endPassiveBrakeTest() {
    const uint32_t gconfToWrite = invertGconfSpreadBit(_brake_saved_gconf, _brake_spread_pin_high);

#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
    ProfileTrace trace;
    trace.expectedWrites = 3;
    trace.targetKnown = true;
    trace.targetGconf = gconfToWrite;
    trace.expectedEffectiveGconf = _brake_saved_gconf;
    trace.targetPwmconf = _brake_saved_pwmconf;
    trace.targetCurrents = _brake_saved_currents;
#endif

    // Keep the pio8.5 write sequence and acknowledgement criterion.
    // Only the representation of the saved mode bit changes.
    for (int attempt = 0; attempt < 3; ++attempt) {
        const uint8_t before = _tmc->IFCNT();
        const bool beforeValid = !_tmc->CRCerror;
        _tmc->IHOLD_IRUN(_brake_saved_currents);
        _tmc->PWMCONF(_brake_saved_pwmconf);
        _tmc->GCONF(gconfToWrite);
        const uint8_t after = _tmc->IFCNT();
        const bool afterValid = !_tmc->CRCerror;
        const bool restoreOk = beforeValid && afterValid &&
                               after == static_cast<uint8_t>(before + 3);
#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
        trace.attempts = attempt + 1;
        trace.ifcntBefore = {before, beforeValid};
        trace.ifcntAfter = {after, afterValid};
        trace.result = !beforeValid ? "ifcnt_before_read_failed" :
                       !afterValid ? "ifcnt_after_read_failed" :
                       restoreOk ? "writes_acknowledged" : "ifcnt_delta_mismatch";
        if (_brake_for_recovery && (restoreOk || attempt == 2)) {
            trace.readback = _captureProfileRegisters();
            _recordProfileTrace(true, trace);
        }
#endif
        if (restoreOk) return true;
    }
    _homed = false;
    _state = MotorState::FAULT;
    _setFaultFlags(_fault_flags | static_cast<uint8_t>(MotorFault::DRIVER_ERROR));
    ESP_LOGE(TAG, "Unable to verify holding-current restore after passive brake test");
    return false;
}
#endif

#ifdef RAVLIGHT_TMC_RECOVERY_DIAGNOSTICS
Tmc2209LocalDriver::RegisterSnapshot Tmc2209LocalDriver::_captureProfileRegisters() {
    RegisterSnapshot snapshot;
    snapshot.uptimeMs = millis();
    snapshot.gconf = {_tmc->GCONF(), !_tmc->CRCerror};
    snapshot.pwmconf = {_tmc->PWMCONF(), !_tmc->CRCerror};
    snapshot.ioin = {_tmc->IOIN(), !_tmc->CRCerror};
    snapshot.drvStatus = {_tmc->DRV_STATUS(), !_tmc->CRCerror};
    snapshot.tstep = {_tmc->TSTEP(), !_tmc->CRCerror};
    return snapshot;
}

void Tmc2209LocalDriver::_recordProfileTrace(bool recovery, const ProfileTrace& trace) {
    portENTER_CRITICAL(&_profileMux);
    (recovery ? _profileDiagnostics.recoveryRestore : _profileDiagnostics.bootProfile) = trace;
    portEXIT_CRITICAL(&_profileMux);
}

Tmc2209LocalDriver::ProfileDiagnostics Tmc2209LocalDriver::profileDiagnostics() {
    portENTER_CRITICAL(&_profileMux);
    const auto copy = _profileDiagnostics;
    portEXIT_CRITICAL(&_profileMux);
    return copy;
}
#endif

// ─── StallGuard — Fase 4 stubs ───────────────────────────────────────────────

void Tmc2209LocalDriver::setStallGuardThreshold(uint8_t threshold) {
    if (_tmc) _tmc->SGTHRS(threshold);
    _sgthrs_active = threshold;
}

uint16_t Tmc2209LocalDriver::getStallGuardResult() {
    return _sg_result;
}

bool Tmc2209LocalDriver::sgLogSnapshot(SgLogEntry* out, uint16_t capacity, uint64_t before,
                                      uint16_t& count, uint64_t& nextBefore) const {
    count = 0;
    nextBefore = 0;
    portENTER_CRITICAL(&_sglog_mux);
    const uint32_t oldest = _sglog_sequence - _sglog_used + 1;
    const uint32_t cursor = static_cast<uint32_t>(before);
    const bool expired = before && (static_cast<uint32_t>(before >> 32) != _sglog_boot ||
                                    cursor <= oldest || cursor > _sglog_sequence);
    if (!expired) {
        const uint16_t skip = before ? _sglog_sequence - cursor + 1 : 0;
        const uint16_t available = _sglog_used - skip;
        count = available < capacity ? available : capacity;
        for (uint16_t i = 0; i < count; ++i) {
            const uint16_t index = (_sglog_head + SGLOG_N - 1 - skip - i) % SGLOG_N;
            out[i] = _sglog[index];
        }
        if (count < available) {
            nextBefore = (static_cast<uint64_t>(_sglog_boot) << 32) |
                         (_sglog_sequence - skip - count + 1);
        }
    }
    portEXIT_CRITICAL(&_sglog_mux);
    return !expired;
}

// Run the motor at homing speed with the stall fault suppressed, so the operator
// can load the shaft and watch SG_RESULT (SGCAL log) without tripping a FAULT.
// We switch to SpreadCycle (StallGuard2) only for the cal run: SG4 in StealthChop
// can saturate near zero on motors already operating close to stall current,
// giving a useless 2-point swing. SG2 in SpreadCycle has a 0-510 range with
// 100-300 points of typical free/loaded difference. Restored in stopStallGuardCal.
void Tmc2209LocalDriver::startStallGuardCal(int8_t direction) {
    if (_state == MotorState::FAULT || _state == MotorState::DRIVER_OFF) {
        ESP_LOGW(TAG, "startStallGuardCal: driver not ready (state=%d)", (int)_state);
        return;
    }
    if (!_stepper || !_tmc) return;
    _sg_cal         = true;
    _stall_isr_flag = false;
    _stepper->setSpeedInHz(_hcfg.speed > 1 ? _hcfg.speed : 1);
    if (direction >= 0) _stepper->runForward();
    else                _stepper->runBackward();
    _state = MotorState::JOGGING;
    ESP_LOGI(TAG, "StallGuard calibration started (SpreadCycle, speed=%u dir=%d)",
             _hcfg.speed, (int)direction);
}

void Tmc2209LocalDriver::stopStallGuardCal() {
    _sg_cal = false;
    if (_stepper) _stepper->stopMove();
    // Restore the operational threshold so DMX moves resume with stall detect.
    if (_tmc)     _tmc->SGTHRS(_hcfg.op_sgthrs);
    _sgthrs_active = _hcfg.op_sgthrs;
    if (_state == MotorState::JOGGING) _state = MotorState::IDLE;
    ESP_LOGI(TAG, "StallGuard calibration stopped");
}

// ─── Status ──────────────────────────────────────────────────────────────────

MotorStatus Tmc2209LocalDriver::getStatus() {
    MotorStatus s;
    s.state       = _state;
    s.position    = _stepper ? _stepper->getCurrentPosition() : 0;
    s.speed       = _stepper ? (_stepper->getCurrentSpeedInMilliHz() / 1000.0f) : 0.0f;
    s.fault_flags = _fault_flags;
    s.sg_result   = _sg_result;
    s.driver_temp = _driver_temp;
    s.homed       = _homed;
    return s;
}

bool Tmc2209LocalDriver::isBusy() {
    return _stepper && _stepper->isRunning();
}

bool Tmc2209LocalDriver::hasFeature(const char* feature) {
    if (strcmp(feature, "stallguard")  == 0) return true;
    if (strcmp(feature, "coolstep")    == 0) return true;
    if (strcmp(feature, "uart_config") == 0) return true;
    return false;
}

// ─── Update loop ─────────────────────────────────────────────────────────────

void Tmc2209LocalDriver::update() {
    if (!_stepper || !_tmc) return;

    // ── State-independent housekeeping ───────────────────────────────────
    // These MUST run regardless of _state. They used to live inside the
    // JOGGING branch — but hardStop() flips JOGGING→IDLE immediately, so
    // the restore below never executed and the 1 M steps/s² boost stayed
    // active for every subsequent move. Symptoms: moveTo commanded ramps
    // the motor can't physically follow → massive lost steps → position
    // counter drifts from the real shaft angle (sweep pre-position ended
    // "at 0" while the load never moved).
    if (_stepper->getCurrentSpeedInMilliHz() == 0) {
        // Clear the anticipated-braking latch so the next jog re-arms.
        if (_limit_braking) _limit_braking = false;
        // Restore operator-configured acceleration after a hardStop boost.
        if (_accel_override_active) {
            _stepper->setAcceleration(_saved_accel);
            _accel_override_active = false;
        }
    }

    // Step pulses are generated by FastAccelStepper in hardware — update() only
    // advances the homing state machine, detects stalls and polls TMC registers.
    if (_state == MotorState::HOMING) {
        _updateHoming();
    } else if (_state == MotorState::JOGGING) {
        // DIAG-based stall during jog. Only honored above the StallGuard4
        // valid velocity (~3000 step/s) — below that SG_RESULT is garbage and
        // DIAG pulses on transients (EMI, cable strain) without a real stall.
        // A jogging operator who's at low speed gets no stall fault; at high
        // speed (~15 cm/s on a typical winch = ~9500 step/s) a real cable
        // jam still trips FAULT and waits for Clear Fault.
        if (_stall_isr_flag) {
            _stall_isr_flag = false;
            int32_t spd_mhz = _stepper->getCurrentSpeedInMilliHz();
            constexpr int32_t STALL_MIN_SPD_MHZ = 3000000;  // 3000 step/s
            bool in_grace = (int32_t)(_stop_grace_until_ms - millis()) > 0;
            // Skip if the override toggle disabled stall detection (operator
            // is redefining limits with an uncalibrated SG profile), or
            // during the post-stop deceleration grace window.
            if (!_sg_cal && !_jog_ignore_stall && !in_grace
                && abs(spd_mhz) >= STALL_MIN_SPD_MHZ) {
                _stopOnOperationalDiag(spd_mhz);
            }
        }
        // Anticipated soft-limit deceleration. Two behaviours in one block:
        //
        //   1) Both soft limits, in normal mode (_jog_ignore_limits=false).
        //      Compute stopping distance d = v² / (2·a) and initiate a
        //      graceful stopMove() *before* the limit, so the motor comes to
        //      rest AT the limit instead of overshooting past it. forceStop
        //      is kept as a hard safety net if the motor is already past.
        //
        //   2) Home-side barrier, even when limits are overridden. Position
        //      0 is the physical drum stop after homing; jogging past it in
        //      the homing direction would crash the mechanism. The barrier
        //      applies regardless of _jog_ignore_limits, so wizard limit-
        //      setting (which uses override=1 for the far-side) still stops
        //      before hitting the drum.
        //
        // Both checks are gated on _homed — before homing the counter is
        // meaningless. accel is snapshot per iteration so a UI change to
        // maxAccel during a jog updates the stopping distance next tick.
        // Anticipated soft-limit + home-side barrier deceleration. Skip
        // entirely once we've already told the ramp generator to stop —
        // otherwise this branch fires stopMove() and its log spam every
        // 1 ms tick, starving the loop task and tripping the watchdog.
        if (_homed && !_limit_braking) {
            int32_t pos     = _stepper->getCurrentPosition();
            int32_t spd_mhz = _stepper->getCurrentSpeedInMilliHz();
            int32_t spd     = spd_mhz / 1000;  // signed steps/s
            uint32_t abs_sp = (uint32_t)abs(spd);
            uint32_t accel  = _stepper->getAcceleration();
            if (accel == 0) accel = 1000;
            uint32_t stop_dist = (uint32_t)((uint64_t)abs_sp * abs_sp / (2 * accel));

            // 1) Normal soft-limit anticipation (both sides, non-override).
            bool triggered = false;
            if (_soft_limits_enabled && !_jog_ignore_limits && spd != 0) {
                bool nearing_top = (spd > 0) && ((int64_t)pos + stop_dist >= _soft_max);
                bool nearing_bot = (spd < 0) && ((int64_t)pos - stop_dist <= _soft_min);
                if (nearing_top || nearing_bot) {
                    bool past_top = pos >= _soft_max && spd > 0;
                    bool past_bot = pos <= _soft_min && spd < 0;
                    if (past_top || past_bot) {
                        _instantStop();   // no ignore_commands trap
                        _state = MotorState::IDLE;
                    } else {
                        _stepper->stopMove();
                    }
                    ESP_LOGI(TAG, "jog braking at limit (pos=%d, stop_dist=%u, range=[%d,%d], spd=%d)",
                             pos, stop_dist, _soft_min, _soft_max, spd);
                    triggered = true;
                }
            }

            // 2) Home-side hard barrier (drum), active when homed — UNLESS the
            //    operator engaged jog override. The barrier normally blocks
            //    motion past 0 in the homing direction to protect the drum,
            //    but that also prevents jogging into the region needed to
            //    capture the correct far/up limit on some rigs. Override
            //    lifts BOTH the soft limits and this barrier (operator takes
            //    responsibility — same contract as manual mode).
            //    _hcfg.direction < 0 → home at min side (block motion past 0
            //    in negative direction); > 0 → home at max side (block past 0
            //    in positive direction).
            if (!triggered && !_jog_ignore_limits && _hcfg.direction != 0 && spd != 0) {
                bool crashing = false;
                if (_hcfg.direction < 0)
                    crashing = (spd < 0) && ((int64_t)pos - stop_dist <= 0);
                else
                    crashing = (spd > 0) && ((int64_t)pos + stop_dist >= 0);
                if (crashing) {
                    _stepper->stopMove();
                    ESP_LOGW(TAG, "jog braking to avoid drum (pos=%d, dir=%d, spd=%d)",
                             pos, (int)_hcfg.direction, spd);
                    triggered = true;
                }
            }
            if (triggered) _limit_braking = true;
        }
    } else {
        // Transition to IDLE when a ramped move reaches its target
        if (_state == MotorState::MOVING &&
            _stepper->getCurrentPosition() == _stepper->targetPos())
            _state = MotorState::IDLE;

        // DIAG-based stall during DMX moves. Same velocity gate as JOG: SG
        // only valid above ~3000 step/s. Most DMX setups run far below that
        // so this effectively disables stall detection during normal
        // operation, but fast moves (e.g. high maxSpeed) still get coverage.
        // Also honour the jog ignore-stall flag and the post-stop grace
        // window: stop() flips state to IDLE immediately even though the
        // motor is still decelerating for hundreds of ms — during that
        // window SG dips linearly with speed and DIAG fires. Without both
        // gates the deceleration ramp of any jog/move would false-trip a
        // stall after every release.
        if (_stall_isr_flag) {
            _stall_isr_flag = false;
            int32_t spd_mhz = _stepper->getCurrentSpeedInMilliHz();
            constexpr int32_t STALL_MIN_SPD_MHZ = 3000000;  // 3000 step/s
            bool in_grace = (int32_t)(_stop_grace_until_ms - millis()) > 0;
            if (!_sg_cal && !_jog_ignore_stall && !in_grace
                && abs(spd_mhz) >= STALL_MIN_SPD_MHZ) {
                _stopOnOperationalDiag(spd_mhz);
            }
        }
    }

    // Periodic TMC register read (driver status + StallGuard result)
    uint32_t now = millis();
    if (now - _last_reg_ms >= REG_READ_INTERVAL_MS) {
        _last_reg_ms = now;
        _readRegisters();

        // Diagnostic ring buffer — one sample per register read. Dumped by
        // the /sglog HTTP route so block-tests can be analysed offline.
        {
            SgLogEntry e{};
            e.t     = now;
            e.sg    = _sg_result;
            e.spd10 = (int16_t)(_stepper->getCurrentSpeedInMilliHz() / 10000);
            e.state = (uint8_t)_state;
            e.flags = (uint8_t)((digitalRead(_diag_pin) ? 1 : 0)
                    | ((int32_t)(_stop_grace_until_ms - now) > 0 ? 2 : 0)
                    | (_jog_ignore_stall ? 4 : 0));
            portENTER_CRITICAL(&_sglog_mux);
            _sglog[_sglog_head] = e;
            _sglog_head = (_sglog_head + 1) % SGLOG_N;
            if (_sglog_used < SGLOG_N) _sglog_used++;
            ++_sglog_sequence;
            portEXIT_CRITICAL(&_sglog_mux);
        }

        // Level-based stall fallback. The DIAG ISR is edge-triggered: if
        // the rising edge lands inside a gate window (post-stop grace,
        // ignore-stall session) the flag is consumed and discarded — but
        // DIAG stays HIGH for the whole stall, so no further edge ever
        // arrives and the stall goes permanently unnoticed. This check
        // re-derives the same condition from the polled register instead
        // (SG_RESULT < 2·SGTHRS for 3 consecutive 100 ms reads) with the
        // same gates as the ISR path.
        bool op_moving = (_state == MotorState::JOGGING || _state == MotorState::MOVING);
        if (op_moving && _sgthrs_active > 0 && !_sg_cal && !_jog_ignore_stall) {
            int32_t spd_mhz = _stepper->getCurrentSpeedInMilliHz();
            bool in_grace = (int32_t)(_stop_grace_until_ms - millis()) > 0;
            uint16_t trip = (uint16_t)_sgthrs_active * 2;
            if (!in_grace && abs(spd_mhz) >= 3000000 && _sg_result < trip) {
                if (++_op_sg_low_count >= 3) {
                    _op_sg_low_count = 0;
                    _instantStop();
                    _setFaultFlags(_fault_flags | (uint8_t)MotorFault::STALL);
                    _state = MotorState::FAULT;
                    ESP_LOGW(TAG, "stall via SG level (sg=%u < %u, spd=%d mHz)",
                             (unsigned)_sg_result, (unsigned)trip, spd_mhz);
                }
            } else {
                _op_sg_low_count = 0;
            }
        } else {
            _op_sg_low_count = 0;
        }
        // StallGuard calibration aid — while the motor moves, stream SG_RESULT at
        // 10 Hz so the operator can read the free-running vs loaded band and pick a
        // homing SGTHRS. Jog free, then hold the shaft and watch SG drop toward 0.
        // Gated behind RAVLIGHT_VERBOSE so it doesn't flood the serial log in a
        // shipping build — useful only when debugging StallGuard tuning.
#ifdef RAVLIGHT_VERBOSE
        if (_state == MotorState::JOGGING || _state == MotorState::HOMING ||
            _state == MotorState::MOVING) {
            ESP_LOGI(TAG, "SGCAL sg=%u state=%d", (unsigned)_sg_result, (int)_state);
        }
#endif
    }
}

// ─── Homing state machine ────────────────────────────────────────────────────

void Tmc2209LocalDriver::_updateHoming() {
    switch (_homing_phase) {

    case HomingPhase::MOVING: {
        // Ignore everything during the brief acceleration ramp.
        if (millis() - _homing_start_ms < HOMING_STALL_BLANK_MS) {
            _stall_isr_flag = false;
            _sg_low_count   = _sg_high_count = 0;
            break;
        }
        const uint16_t trip_thr = (uint16_t)_hcfg.sgthrs * 2;
        const uint32_t elapsed  = millis() - _homing_start_ms;

        // Phase A — settle: wait for SG to first cross above trip_thr (free-run
        // reached). This adapts to the actual PWM autoscale settling time and
        // avoids false-tripping on the rising transient (SG climbs 0 → ~180).
        if (!_homing_settled) {
            // DIAG fires throughout the transient (TMC2209 hardware asserts it
            // whenever SG*2 ≤ SGTHRS, which includes SG=0 at startup). Drop the
            // flag every tick during settle so phase B doesn't inherit a stale
            // assertion.
            _stall_isr_flag = false;
            if (_sg_result > trip_thr) {
                if (++_sg_high_count >= HOMING_SG_SETTLE_HIGH) {
                    _homing_settled = true;
                    _sg_low_count   = 0;
                    ESP_LOGI(TAG, "homing settled (sg=%u thr=%u after %ums)",
                             (unsigned)_sg_result, (unsigned)trip_thr, elapsed);
                }
            } else {
                _sg_high_count = 0;
            }
            // Settle timeout: motor never reached free-run regime → already at
            // end stop. Same trip path as a normal stall.
            if (elapsed < HOMING_SETTLE_TIMEOUT_MS) break;
            ESP_LOGW(TAG, "homing settle timeout (sg=%u) — assuming start at end stop",
                     (unsigned)_sg_result);
        } else {
            // Phase B — watch: SG previously settled high, now look for a drop.
            // We deliberately ignore _stall_isr_flag here: TMC2209 DIAG fires on
            // any momentary SG dip below SGTHRS, including transient bumps
            // during normal free-run motion, which produced false trips. SG
            // polling with debounce (5 × 100 ms = 500 ms sustained low) is the
            // single source of truth — no hardware fallback.
            _stall_isr_flag = false;
            if (_sg_result > 0 && _sg_result < trip_thr) {
                if (++_sg_low_count < HOMING_SG_DEBOUNCE) break;
                // 5 consecutive low samples → real stall, fall through to trip.
            } else {
                _sg_low_count = 0;
                break;
            }
        }

        // End stop reached — stop and zero the position counter
        _stepper->forceStopAndNewPosition(0);

        // Restore operational current. SGTHRS goes to 0 (DIAG disarmed) for
        // post-homing operation — operSgthrs is only valid at the homing
        // velocity, and DMX moves typically run at a different speed where
        // the natural SG_RESULT band sits below the calibrated trip, causing
        // false stall faults. Hardware OT/short protections remain.
        setRunCurrent(_run_ma);
        _tmc->SGTHRS(0);
        _sgthrs_active = 0;

        // Retreat from the end stop (ramped move away from it)
        _stepper->setSpeedInHz(_hcfg.speed > 1 ? _hcfg.speed : 1);
        _stepper->moveTo((int32_t)_hcfg.backoff_steps * -_hcfg.direction);
        _homing_phase = HomingPhase::BACKOFF;
        ESP_LOGI(TAG, "stall at end stop (sg=%u thr=%u) — backing off %u steps",
                 (unsigned)_sg_result, (unsigned)trip_thr, _hcfg.backoff_steps);
        break;
    }

    case HomingPhase::BACKOFF:
        if (_stepper->getCurrentPosition() == _stepper->targetPos()) {
            // Back-off complete — declare home at current position. Clear any
            // residual DIAG assertion: while the operator was holding the rope
            // during the stall trip the TMC2209 kept asserting DIAG, and on
            // transition to IDLE the regular update() path would otherwise
            // immediately re-promote that flag to a STALL fault.
            _stepper->setCurrentPosition(0);
            _stall_isr_flag = false;
            _homed          = true;
            _homing_phase   = HomingPhase::IDLE;
            _state          = MotorState::IDLE;
            // Re-arm operational stall detection. The stall-trip handler
            // above zeroed SGTHRS for the backoff leg; historically it was
            // left at 0 after homing ("operSgthrs only valid at homing
            // speed") which silently disabled ALL hardware stall detection
            // for the rest of the session. With the sweep-derived,
            // speed-aware op threshold that concern no longer applies.
            _tmc->SGTHRS(_hcfg.op_sgthrs);
            _sgthrs_active = _hcfg.op_sgthrs;
            ESP_LOGI(TAG, "homing complete — home set, op SGTHRS=%u re-armed",
                     (unsigned)_hcfg.op_sgthrs);
        }
        break;

    case HomingPhase::IDLE:
    default:
        break;
    }
}

// ─── Private helpers ─────────────────────────────────────────────────────────

void Tmc2209LocalDriver::_stopOnOperationalDiag(int32_t speed_mhz) {
    // DIAG can also indicate a driver fault. Keep the stop and holding
    // torque even when operational StallGuard is disabled.
    _instantStop();
    _state = MotorState::FAULT;

    GSTAT_t gstat{};
    gstat.sr = _tmc->GSTAT();
    const bool gstat_valid = !_tmc->CRCerror;
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    _observeDriverStatus(gstat.sr, gstat_valid);
#endif
    TMC2208_n::DRV_STATUS_t drv_status{};
    drv_status.sr = _tmc->DRV_STATUS();
    const bool drv_status_valid = !_tmc->CRCerror;
    const bool driver_error =
        (gstat_valid && (gstat.drv_err || gstat.uv_cp)) ||
        (drv_status_valid && (drv_status.ot || drv_status.s2ga || drv_status.s2gb ||
                              drv_status.s2vsa || drv_status.s2vsb));
    if (_sgthrs_active == 0 || driver_error) {
        _setFaultFlags(_fault_flags | static_cast<uint8_t>(MotorFault::DRIVER_ERROR));
        if (_invalidate_home_on_stall) _homed = false;
    } else {
        _setFaultFlags(_fault_flags | static_cast<uint8_t>(MotorFault::STALL));
    }
    Serial.printf("[TMC2209] DIAG stop: speed=%ld mHz SGTHRS=%u GSTAT=0x%02x (%s) DRV_STATUS=0x%08lx (%s)\n",
                  static_cast<long>(speed_mhz), static_cast<unsigned>(_sgthrs_active),
                  static_cast<unsigned>(gstat.sr), gstat_valid ? "valid" : "CRC error",
                  static_cast<unsigned long>(drv_status.sr), drv_status_valid ? "valid" : "CRC error");
}

void Tmc2209LocalDriver::_readRegisters() {
    _sg_result = _tmc->SG_RESULT();

    // DRV_STATUS UART reads are corrupted by coil EMI while the motor is
    // moving — individual otpw/ot/s2g* accessors can flip a fault flag on
    // even when the chip is cold and the wiring fine. The TMC2209 protects
    // itself in hardware (157 °C thermal shutdown, short-circuit
    // auto-disable) regardless of our software supervision, so we skip the
    // poll while running and add a 500 ms cool-off after motion ends.
    bool moving = _stepper && _stepper->isRunning();
    if (moving) {
        _last_motion_ms = millis();
        return;
    }
    if (millis() - _last_motion_ms < 500) {
        return;
    }

#ifdef RAVLIGHT_TMC_STEALTHCHOP_TRIAL
    const bool stealth = _tmc->stealth();
    _idleChopperMode.store(_tmc->CRCerror ? ChopperMode::UNKNOWN :
                          stealth ? ChopperMode::STEALTH_CHOP : ChopperMode::SPREAD_CYCLE,
                          std::memory_order_relaxed);
#endif

    // DRV_STATUS bit layout differs across TMC chips — use the library's
    // chip-specific accessors instead of raw bit math. The accessors include
    // CRC retry logic that a direct register read does not.
    bool otpw = _tmc->otpw();   // over-temperature pre-warning (~120 °C)
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    const bool warningValid = !_tmc->CRCerror;
#endif
    bool ot   = _tmc->ot();     // over-temperature shutdown (~150 °C)
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    if (warningValid && !_tmc->CRCerror) {
        using namespace OrionMotorEvents;
        const uint8_t thermal = ot ? 2 : otpw ? 1 : 0;
        if (thermal != _thermalLevel) {
            record(thermal == 2 ? Kind::THERMAL_SHUTDOWN : thermal == 1 ? Kind::THERMAL_WARNING : Kind::THERMAL_CLEAR,
                   thermal, _thermalLevel);
            _thermalLevel = thermal;
        }
    }
#endif
    bool s2ga = _tmc->s2ga();   // short to GND phase A
    bool s2gb = _tmc->s2gb();   // short to GND phase B
    // GSTAT.uv_cp — charge-pump supply undervoltage. Unlike the DRV_STATUS
    // bits above, GSTAT is a write-to-clear latch: it stays set once tripped
    // (e.g. a 12 V supply on a board meant for 24 V — boots the ESP32 fine
    // but the driver never had enough voltage to run) until we clear it, so
    // capture it into our own _fault_flags and clear the chip latch here.
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    GSTAT_t general{};
    general.sr = _tmc->GSTAT(); // Same UART read as the original uv_cp() accessor.
    _observeDriverStatus(general.sr, !_tmc->CRCerror);
    const bool undervoltage = general.uv_cp;
#else
    const bool undervoltage = _tmc->uv_cp();
#endif
    if (undervoltage) {
        _setFaultFlags(_fault_flags | (uint8_t)MotorFault::UNDERVOLTAGE);
        ESP_LOGE(TAG, "TMC2209 undervoltage on charge pump — check supply voltage");
        _tmc->GSTAT(0x07);   // clear reset/drv_err/uv_cp latches
    }

    if (ot || otpw) {
        _setFaultFlags(_fault_flags | (uint8_t)MotorFault::OVERTEMP);
        _driver_temp  = ot ? 150 : 120;
        if (ot) ESP_LOGE(TAG, "TMC2209 overtemperature shutdown!");
    } else {
        _setFaultFlags(_fault_flags & ~(uint8_t)MotorFault::OVERTEMP);
    }
    if (s2ga || s2gb) {
        _setFaultFlags(_fault_flags | (uint8_t)MotorFault::OVERCURRENT);
        ESP_LOGE(TAG, "TMC2209 short to GND detected");
    }
}

void Tmc2209LocalDriver::_setFaultFlags(uint8_t flags) {
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    const uint8_t previous = _fault_flags;
#endif
    _fault_flags = flags;
#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
    using namespace OrionMotorEvents;
    if (flags & ~previous) record(Kind::MOTOR_FAULT, flags, previous);
    if (previous & ~flags) record(Kind::MOTOR_FAULT_CLEAR, flags, previous);
#endif
}

#ifdef RAVLIGHT_ORION_MOTOR_EVENT_LOG
void Tmc2209LocalDriver::_observeDriverStatus(uint8_t gstat, bool valid, bool atBoot) {
    if (!valid) return;
    const bool reset = (gstat & 1) != 0;
    if (reset && !_tmcResetSeen) {
        using namespace OrionMotorEvents;
        record(atBoot ? Kind::TMC_RESET_AT_BOOT : Kind::TMC_RESET, gstat);
    }
    // The existing recovery checks own GSTAT. Logging never clears its latch.
    _tmcResetSeen = reset;
}
#endif

void IRAM_ATTR Tmc2209LocalDriver::_diagIsrCb(void* arg) {
    Tmc2209LocalDriver* drv = static_cast<Tmc2209LocalDriver*>(arg);
    drv->_stall_isr_flag = true;
}

#endif // RAVLIGHT_HAS_MOTOR
