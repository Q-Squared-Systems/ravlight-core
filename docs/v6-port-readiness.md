# Orion v6 SDK port readiness

## Preflash update

The network fixes, reset reporting, and Motor UI fix from `brake-net2` are
now in the port. Both v6 targets built successfully. The port reports
`2.24.0-pio8.1` and has a separate OTA image under `release/orion/v2.24.0-pio8.1/`.
Esptool verified both application images. The port image uses 1,423,856 bytes
out of the 1,638,400-byte OTA slot in the supplied 2.24.1 full image.

Operational DIAG events still stop motion and preserve hold. With StallGuard
disabled, or when a driver fault is confirmed by the registers, the firmware
reports a driver fault. The serial log records the register readings and their
CRC status. Hardware timing and the reboot hold behavior remain to be tested.

The sections below record the original audit and the device test sequence.

Reviewed November 19, 2026, against `codex/v6-platformio8-port` at `cce3757`.
Scope: local source and build review. No device was contacted or flashed.

## Decision

The SDK port compiles, but carry forward the missing fixes and distinguish the
test firmware before installing it on Chandelier 1. Compilation does not verify
motor holding torque, pulse timing, Ethernet recovery, or LED refresh under load.

## Verified here

| Target | Result | Static RAM | Linked flash usage |
| --- | --- | --- | --- |
| `led_lifter_v6_orion_pio8` | Pass | 110,640 bytes | 1,388,354 / 1,638,400 bytes |
| `led_lifter_v6_orion` | Pass | 109,640 bytes | 1,243,765 / 1,638,400 bytes |

Builds used the locally cached dependencies, including FastAccelStepper 2.0.1,
ESPAsyncWebServer 3.6.0 (`feb92de`), and ArduinoJson 6.21.6. The port build uses
PlatformIO Espressif32 8.0.0 and Arduino-ESP32 3.3.12. Its image identifies
ESP-IDF v5.5.5. This is verification of this dependency set, not a fresh dependency
resolution from the network.

The port application image is 1,416,800 bytes, leaving 221,600 bytes in the
repository's 0x190000-byte OTA slot. Esptool 5.4.0 verified its checksum and
validation hash. SHA-256:
`73bb0a92f3e687d647b32eff79667526485e7f047c3ec631d94476dc8dd70100`.

Build logs: `/private/tmp/q2lifter-pio8-review-build.log` and
`/private/tmp/q2lifter-v6-review-build.log`.

## Work before the device test

1. **Bring forward the earlier network fixes.** The saved `brake-net2` patch
   contains changes absent here: bounded request-owned JSON body collection,
   complete-body parsing, protected discovery updates and reader snapshots,
   nonblocking DMX monitor snapshots, and response-allocation checks.
   `src/webserver_manager.cpp` still uses `AsyncCallbackJsonWebHandler` for
   `/api/config`; the resolved library allocates exactly `total` bytes, copies
   chunks without local bounds checks, and parses without an explicit length.
   `/upload_config` parses each body callback independently. Discovery still
   exposes and mutates the shared vector without synchronization. Review and
   adapt the patch rather than replacing the port's ESP-NOW callback changes.
2. **Restore observability.** `/api/status` has no reset reason in this branch.
   Restore the `brake-net2` reset fields before investigating another reboot.
   Preserve a serial log for the first SDK boot and any reset/hold testing.
3. **Restore the UI scope fix.** In `data/orion/fixture.js`, `oob` is declared
   inside `if (!_dragging)` around line 1077 but used outside it around line 1112.
   A live position reaches a `ReferenceError`. The earlier patch moves the
   declaration outside that block. Verify Motor UI polling and Passive Brake Test
   visibility after correcting it; do not execute the brake test as part of UI QA.
4. **Identify and package the test build separately.** Both targets inherit
   `custom_fw_name = orion_led_lifter_v6` and `FW_VERSION = 2.24.0`; both write
   the same versioned release filenames. Building legacy second overwrote the
   port's release copies during this review. Give the test a distinct version
   and artifact filename, preserve its board compatibility identity, and retain
   an explicit exclusion from release/latest. Pin the reviewed dependencies so
   later builds do not silently change the motor/network stack. The port image
   remains available at `.pio/build/led_lifter_v6_orion_pio8/firmware.bin`.
5. **Resolve or explicitly bound the existing Manual Mode defect.** Operational
   mode sets SGTHRS to zero in Manual Mode, but the jog and DMX DIAG branches in
   `Tmc2209LocalDriver.cpp` do not gate on that setting. They can still latch a
   stall above 3,000 steps/second. This predates the SDK port. Correct the mode
   handling and capture fault-source/register evidence before using this test
   to evaluate the previously observed descent faults; keep homing behavior and
   electrical/thermal fault handling distinct.

The earlier patch is currently outside this checkout at
`/Users/qualleyiv/Documents/ChatGPT/Winder Wonderland/firmware/chandelier-1-brake-net2/source.patch`.

## Chandelier 1 test sequence after those changes

1. Read the board identity at its known address `10.0.2.151`, back up its current
   configuration, and retain the known-good 2.24.1 application image. Verify the
   actual installed partition layout and update compatibility; repository slot
   sizes alone do not establish the board's layout. Use an application image for
   OTA, not a merged full-flash image.
2. Establish a supported load and disarmed DMX baseline before the reboot.
   Record current settings, motor faults, uptime, and link/DMX counters. Capture
   the new build identity and reset reason after the update.
3. Check saved configuration, TMC UART communication, driver enable/hold,
   Ethernet IP and Art-Net reception, API responses, and Motor UI first.
4. Test slow moves, reversals and controlled deceleration, then the intended cue
   speeds with LEDs and HTTP polling active. Verify stop and DMX-loss behavior.
   Compare step pulse timing and LED waveforms with the legacy build.
5. Exercise all four LED outputs together with the motor. Under the resolved
   IDF5 FastAccelStepper library, automatic selection tries RMT first; legacy
   selection tries MCPWM/PCNT first. The LED port also uses RMT. No definite
   allocation conflict was established, but this is a real backend change that
   needs combined testing. Verify LED pulse widths, inter-frame reset/latch gap,
   refill behavior and timeout counts under load. ESP-IDF documents shared RMT
   clock constraints and interrupt/encoder timing limitations in its
   [RMT guide](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/api-reference/peripherals/rmt.html).
6. Check link disconnect/reconnect, sustained DMX with status polling/discovery,
   free heap trend and thermal flags. Exercise retained enable across supported
   software resets separately with the load supported. Do not infer power-loss
   or brownout holding behavior from a software-reset test.

PSRAM remains disabled in both targets; enabling it is separate work and is not
required for this test. No runtime measurements or current fleet state were
established by this review.
