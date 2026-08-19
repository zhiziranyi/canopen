# No-INA240 Safe FOC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the STM32 CANopen servo operate without an INA240 while preventing sustained energized stall conditions.

**Architecture:** Keep CiA402, CANopen, AS5600 feedback, and voltage-mode FOC. Introduce an explicit compile-time current-sense capability. In the default no-sensor build, ADC calibration, current reconstruction, current display, and ADC overcurrent logic are bypassed; a command/velocity-based stall watchdog disables PWM and latches a fault if a meaningful speed command fails to create motion.

**Tech Stack:** STM32F407 HAL, CANopenNode, PlatformIO, C99, native C tests.

---

### Task 1: Define and test the no-sensor safety policy

**Files:**
- Create: `src/stall_guard.h`
- Create: `src/stall_guard.c`
- Modify: `test/native/test_main.c`

- [ ] **Step 1: Write failing native tests**

Add tests that require a guard to stay idle for a zero/small command, accumulate only while the command and applied voltage are meaningful but feedback speed is too low, trip after 300 ms, and clear on feedback motion.

```c
stall_guard_init(&guard);
assert(!stall_guard_step(&guard, 500.0f, 0.8f, 0.0f, 0.001f));
for (unsigned i = 0; i < 299; ++i) {
    assert(!stall_guard_step(&guard, 500.0f, 0.8f, 0.0f, 0.001f));
}
assert(stall_guard_step(&guard, 500.0f, 0.8f, 0.0f, 0.001f));
```

- [ ] **Step 2: Run the native test and verify it fails because `stall_guard.h` is absent**

Run: `powershell -ExecutionPolicy Bypass -File tools\\run_native_tests.ps1`

- [ ] **Step 3: Implement the smallest reusable guard**

Expose `stall_guard_init()` and `stall_guard_step()`. Make thresholds explicit arguments so the policy can be tested without STM32 HAL.

- [ ] **Step 4: Re-run native tests**

Run: `powershell -ExecutionPolicy Bypass -File tools\\run_native_tests.ps1`

### Task 2: Make current sensing an explicit optional capability

**Files:**
- Modify: `src/config.h`
- Modify: `src/board.c`
- Modify: `src/board.h`
- Modify: `src/motor.h`
- Modify: `src/motor.c`

- [ ] **Step 1: Add a default-off capability switch**

Define `CURRENT_SENSE_PRESENT` as `0` by default and reject a current-loop build without a sensor:

```c
#ifndef CURRENT_SENSE_PRESENT
#define CURRENT_SENSE_PRESENT 0
#endif

#if FOC_CURRENT_LOOP_ENABLE && !CURRENT_SENSE_PRESENT
#error "FOC current loop requires CURRENT_SENSE_PRESENT=1"
#endif
```

- [ ] **Step 2: Gate all ADC-specific initialization and sampling**

When `CURRENT_SENSE_PRESENT == 0`, do not configure PA3 as analog input, do not initialize ADC1, do not enable its IRQ, and make `adc_start_sample()` a successful no-op. Do not run zero-current calibration.

- [ ] **Step 3: Keep one unambiguous motor API for diagnostics**

Add `int motor_has_current_sense(void);`. In no-sensor mode, `motor_get_current_*()` must return zero only as a compatibility value and callers must use `motor_has_current_sense()` before displaying current.

- [ ] **Step 4: Compile the firmware**

Run: `& "$env:USERPROFILE\\.platformio\\penv\\Scripts\\pio.exe" run -e black_f407zg`

### Task 3: Add no-sensor stall shutdown and safe alignment limits

**Files:**
- Modify: `src/config.h`
- Modify: `src/motor.c`
- Modify: `src/cia402.c`

- [ ] **Step 1: Add safe no-sensor constants**

Use a low voltage limit and a short alignment hold in no-sensor builds. Define separate stall thresholds for command magnitude, applied Vq, measured speed, and timeout.

- [ ] **Step 2: Wire the guard into the 1 kHz velocity control path**

Only evaluate it when CiA402 has enabled the motor and the velocity command magnitude is at least the configured start threshold. On trip call `motor_latch_fault(MOTOR_FAULT_STALL)`; this path must clear PWM and pull driver EN low through the existing latch routine.

- [ ] **Step 3: Map the new fault to CiA402**

Expose the stall condition as a distinct device error in `603Fh`, set the fault statusword bit, and keep fault reset behavior unchanged.

- [ ] **Step 4: Extend native tests for fault mapping if it is pure logic, then run all native tests**

Run: `powershell -ExecutionPolicy Bypass -File tools\\run_native_tests.ps1`

### Task 4: Make the user interface and documentation match the hardware

**Files:**
- Modify: `src/main.c`
- Modify: `README.md`
- Modify: `开发参考方案.md`

- [ ] **Step 1: Display current as unavailable when absent**

Use `motor_has_current_sense()` so the TFT/serial diagnostic renders `Iq: N/A` rather than a floating ADC-derived number.

- [ ] **Step 2: Document the no-sensor operating limits**

State that voltage-mode FOC has no measured overcurrent protection; it must use a current-limited bench supply for first spin testing, conservative voltage, and the software stall timeout.

### Task 5: Verification and version history

**Files:**
- Modify: `test/native/test_main.c`
- Modify: `src/*.c`
- Modify: `src/*.h`

- [ ] **Step 1: Run native tests**

Run: `powershell -ExecutionPolicy Bypass -File tools\\run_native_tests.ps1`

- [ ] **Step 2: Perform a clean PlatformIO firmware build**

Run: `& "$env:USERPROFILE\\.platformio\\penv\\Scripts\\pio.exe" run -e black_f407zg`

- [ ] **Step 3: Inspect the final diff and commit**

Run: `git diff --check; git status --short`

Commit message: `feat: add safe no-current-sensor foc mode`

### Review checklist

- [ ] The default build neither configures nor reads PA3/ADC1.
- [ ] The default build cannot enable the FOC current PI loop.
- [ ] A blocked motor under a meaningful speed command is disabled within the configured timeout.
- [ ] CiA402, SDO target velocity, PDO feedback, and heartbeat remain unchanged.
- [ ] Native tests and `black_f407zg` build both exit successfully.
