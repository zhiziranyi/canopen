# CANopen FOC Control Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the STM32F407 CANopen CiA402 drive produce deterministic 20 kHz FOC PWM, execute safe standard CiA402 enable/motion commands, and expose diagnosable motor/encoder faults.

**Architecture:** Keep CANopenNode and the current object dictionary. Extract hardware-independent encoder math, CiA402 power-state transitions, and motion profiling into small C modules tested with host GCC; keep TIM1 ISR limited to FOC/PWM/protection, use interrupt-driven AS5600 sampling at 1 kHz, and keep CANopen/CiA402 orchestration in the main loop.

**Tech Stack:** C11, STM32CubeF4 HAL, PlatformIO `ststm32`, CANopenNode v4, Strawberry GCC host tests, PowerShell test runner.

---

## File structure

- `src/encoder_math.h/.c`: hardware-independent AS5600 unwrap, accumulated position, and filtered velocity.
- `src/cia402_sm.h/.c`: hardware-independent CiA402 power state transitions and base statusword.
- `src/motion_profile.h/.c`: bounded trapezoidal/triangular velocity command generator.
- `test/native/test_main.c`: direct behavior tests for PID, FOC math, encoder math, CiA402 states, and motion profile.
- `tools/run_native_tests.ps1`: compiles the pure C modules with `gcc -Wall -Wextra -Werror` and executes the test binary.
- `src/encoder.h/.c`: interrupt-driven AS5600 transport and health counters; delegates math to `encoder_math`.
- `src/motor.h/.c`: deterministic PWM/FOC startup, alignment verification, motor fault codes, and use of `motion_profile`.
- `src/board.c`: TIM1 update frequency, I2C IRQ plumbing, ADC timing, and watchdog initialization.
- `src/cia402.c`: use `cia402_sm`, map motor faults, validate modes, and clear errors correctly.
- `src/main.c`: enforce initialization failures, healthy-loop watchdog refresh, and improved diagnostics.
- `platformio.ini`: compile application code with warnings as errors and include new modules.
- `tools/drive_test.py`, `README.md`: repeatable CANopen enable/velocity/disable verification.

### Task 1: Add the native test harness and FOC/PID regression tests

**Files:**
- Create: `test/native/test_main.c`
- Create: `tools/run_native_tests.ps1`
- Test: `src/foc.c`, `src/pid.c`

- [ ] **Step 1: Write failing host tests for zero-vector SVPWM, output bounds, Clarke/Park axes, PID saturation, and PID reset**

Create a small C runner with a `CHECK_TRUE` macro, `nearly_equal()` helper, one function per behavior, and a final nonzero exit when any check fails. The test must assert:

```c
foc_inverse_park_svpwm(0.0f, 0.0f, 0.0f, 1.0f, 12.0f, &du, &dv, &dw);
CHECK_NEAR(du, 0.5f, 1.0e-6f);
CHECK_NEAR(dv, 0.5f, 1.0e-6f);
CHECK_NEAR(dw, 0.5f, 1.0e-6f);

foc_clarke_park(1.0f, -0.5f, -0.5f, 0.0f, 1.0f, &id, &iq);
CHECK_NEAR(id, 1.0f, 1.0e-5f);
CHECK_NEAR(iq, 0.0f, 1.0e-5f);

pid_init(&pid, 2.0f, 1.0f, 0.0f, -1.0f, 1.0f, 0.001f);
CHECK_NEAR(pid_update(&pid, 100.0f), 1.0f, 1.0e-6f);
pid_reset(&pid);
CHECK_NEAR(pid.integral, 0.0f, 1.0e-6f);
```

- [ ] **Step 2: Add the PowerShell runner and verify the tests fail before all planned modules exist**

The runner invokes:

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Isrc `
    test/native/test_main.c src/foc.c src/pid.c `
    src/encoder_math.c src/cia402_sm.c src/motion_profile.c `
    -lm -o .pio/native-tests.exe
& .\.pio\native-tests.exe
```

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: compilation fails because `encoder_math.c`, `cia402_sm.c`, and `motion_profile.c` do not exist yet. This is the RED evidence for the extracted modules.

- [ ] **Step 3: Temporarily compile only FOC/PID tests and confirm they pass**

Run a direct GCC command containing only `test_main.c`, `src/foc.c`, and `src/pid.c`.  
Expected: exit code 0 and `native tests: PASS` for the existing math behaviors.

- [ ] **Step 4: Commit the test harness**

```powershell
git add test/native/test_main.c tools/run_native_tests.ps1
git commit -m "test: add native motor-control harness"
```

### Task 2: Extract encoder unwrap and velocity math

**Files:**
- Create: `src/encoder_math.h`
- Create: `src/encoder_math.c`
- Modify: `test/native/test_main.c`

- [ ] **Step 1: Add failing encoder tests**

Define the intended API in the test:

```c
encoder_math_t enc;
encoder_math_init(&enc, 4095u);
encoder_math_update(&enc, 0u, 0.001f);
CHECK_TRUE(encoder_math_position(&enc) == 1);
encoder_math_update(&enc, 4095u, 0.001f);
CHECK_TRUE(encoder_math_position(&enc) == 0);
CHECK_TRUE(fabsf(encoder_math_velocity(&enc)) < 1000.0f);
```

Also test a normal `100 -> 104` update yields position `4` and positive filtered velocity.

- [ ] **Step 2: Run the native tests and verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: compiler reports missing `encoder_math.h` or undefined `encoder_math_*` symbols.

- [ ] **Step 3: Implement `encoder_math`**

Use this public shape:

```c
typedef struct {
    uint16_t previous_raw;
    int32_t position;
    float velocity;
    uint8_t initialized;
} encoder_math_t;

void encoder_math_init(encoder_math_t *state, uint16_t raw);
void encoder_math_update(encoder_math_t *state, uint16_t raw, float dt_s);
int32_t encoder_math_position(const encoder_math_t *state);
float encoder_math_velocity(const encoder_math_t *state);
void encoder_math_zero_position(encoder_math_t *state);
```

Wrap deltas outside ±2048 by adding/subtracting 4096, calculate `instant_velocity = delta / dt_s`, and filter with `0.9 * old + 0.1 * instant`.

- [ ] **Step 4: Run native tests and verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: encoder tests pass; the run may still fail only for the not-yet-created CiA402/profile modules.

- [ ] **Step 5: Commit**

```powershell
git add src/encoder_math.c src/encoder_math.h test/native/test_main.c
git commit -m "refactor: extract encoder position math"
```

### Task 3: Extract and test the CiA402 power-state machine

**Files:**
- Create: `src/cia402_sm.h`
- Create: `src/cia402_sm.c`
- Modify: `test/native/test_main.c`

- [ ] **Step 1: Add failing state transition tests**

Tests must create a state machine at Switch On Disabled and assert:

```c
cia402_sm_init(&sm);
CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0240u);
cia402_sm_step(&sm, 0x0006u, false);
CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0231u);
cia402_sm_step(&sm, 0x0007u, false);
CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0233u);
cia402_sm_step(&sm, 0x000Fu, false);
CHECK_TRUE(cia402_sm_operation_enabled(&sm));
CHECK_TRUE(cia402_sm_statusword(&sm) == 0x0237u);
cia402_sm_step(&sm, 0x000Bu, false);
CHECK_TRUE(!cia402_sm_operation_enabled(&sm));
```

Also assert a hardware fault enters Fault, and fault reset occurs only on a bit7 rising edge.

- [ ] **Step 2: Run and verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: missing `cia402_sm` API.

- [ ] **Step 3: Implement the pure state machine**

The struct stores `cia402_state_t state` and `uint16_t previous_controlword`. `cia402_sm_step()` handles Shutdown, Switch On, Enable Operation, Disable Voltage, Quick Stop, hardware Fault, and bit7 edge reset. `cia402_sm_statusword()` returns the base CiA402 bits plus Remote bit9.

- [ ] **Step 4: Run and verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: CiA402 tests pass; only profile tests may remain absent.

- [ ] **Step 5: Commit**

```powershell
git add src/cia402_sm.c src/cia402_sm.h test/native/test_main.c
git commit -m "refactor: isolate CiA402 power state machine"
```

### Task 4: Replace unsafe position-profile arithmetic

**Files:**
- Create: `src/motion_profile.h`
- Create: `src/motion_profile.c`
- Modify: `test/native/test_main.c`

- [ ] **Step 1: Add failing profile tests**

Use this API and cover stationary, forward, reverse, acceleration limiting, and braking:

```c
motion_profile_t profile;
motion_profile_init(&profile, 2000.0f, 10000.0f, 10000.0f);
motion_profile_set_target(&profile, 0, 1000);
CHECK_TRUE(motion_profile_step(&profile, 0, 0.001f) > 0.0f);
CHECK_TRUE(motion_profile_step(&profile, 2000, 0.001f) < 0.0f);
motion_profile_set_target(&profile, 100, 100);
CHECK_NEAR(motion_profile_step(&profile, 100, 0.001f), 0.0f, 1.0e-6f);
```

- [ ] **Step 2: Run and verify RED**

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: missing `motion_profile` API.

- [ ] **Step 3: Implement bounded profile generation**

Store target, current velocity, velocity limit, acceleration, and deceleration as finite values. Each step computes signed remaining distance, braking velocity `sqrtf(2 * deceleration * abs(distance))`, clamps desired speed to the lower of braking speed and velocity limit, then slew-limits the command by `acceleration * dt` or `deceleration * dt`. Return zero inside a four-count target window once speed is below 50 counts/s.

- [ ] **Step 4: Run the complete native suite and verify GREEN**

Run: `powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1`  
Expected: `native tests: PASS`, exit code 0, and no compiler warnings.

- [ ] **Step 5: Commit**

```powershell
git add src/motion_profile.c src/motion_profile.h test/native/test_main.c tools/run_native_tests.ps1
git commit -m "feat: add bounded position motion profile"
```

### Task 5: Make AS5600 sampling non-blocking and observable

**Files:**
- Modify: `src/encoder.h`
- Modify: `src/encoder.c`
- Modify: `src/board.c`
- Modify: `src/board.h`

- [ ] **Step 1: Add compile-time contract checks to the host/firmware interfaces**

Declare:

```c
int encoder_start_sample(void);
void encoder_sample_complete_isr(void);
void encoder_sample_error_isr(void);
int encoder_is_healthy(void);
uint32_t encoder_get_error_count(void);
```

Run the firmware build before implementation.  
Expected: link/compile failure after board callbacks reference the missing implementations.

- [ ] **Step 2: Implement interrupt-driven I2C transport**

Keep the single blocking probe in `encoder_init()` before interrupts start. Runtime uses `HAL_I2C_Mem_Read_IT()` into a two-byte static buffer. Completion masks the AS5600 value with `0x0FFF`, increments a sample sequence, and clears pending state. Error callback clears pending state and increments an error counter.

`encoder_update_1k()` consumes only a new sequence through `encoder_math_update()`. Twenty consecutive stale ticks mark the encoder unhealthy; a successful new sample clears the stale counter.

- [ ] **Step 3: Add I2C interrupt handlers and callbacks**

Enable `I2C1_EV_IRQn` and `I2C1_ER_IRQn` at priority 3, implement both IRQ handlers, route `HAL_I2C_MemRxCpltCallback()` and `HAL_I2C_ErrorCallback()` to the encoder only when `hi2c->Instance == I2C1`.

- [ ] **Step 4: Verify tests and firmware build**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

Expected: both commands exit 0.

- [ ] **Step 5: Commit**

```powershell
git add src/encoder.c src/encoder.h src/board.c src/board.h
git commit -m "refactor: sample AS5600 without blocking control ISR"
```

### Task 6: Repair TIM1 FOC scheduling and alignment validation

**Files:**
- Modify: `src/board.c`
- Modify: `src/config.h`
- Modify: `src/motor.h`
- Modify: `src/motor.c`
- Modify: `src/main.c`

- [ ] **Step 1: Add a failing source-level scheduling regression check**

Extend the PowerShell test runner to fail unless `src/motor.c` contains `HAL_TIM_PWM_Start(` and `HAL_TIM_Base_Start_IT(&htim1)`, and to fail if it contains `HAL_TIM_PWM_Start_IT(&htim1`. Run it before modifying motor startup.  
Expected: runner fails on the forbidden `HAL_TIM_PWM_Start_IT` call.

- [ ] **Step 2: Start PWM and update interrupts correctly**

Set TIM1 `RepetitionCounter = 1` for one update per complete center-aligned PWM cycle. In `motor_init()`, check each `HAL_TIM_PWM_Start()` return, then check `HAL_TIM_Base_Start_IT(&htim1)`. Record initialization failure in `motor_fault_t` and keep EN low.

Increment a volatile `s_control_update_count` at entry to `motor_current_loop_isr()` and expose it through `motor_get_control_update_count()`.

- [ ] **Step 3: Make alignment return a verified result**

Change `motor_align_foc()` to return `int`. Drive `Vd=FOC_ALIGN_VOLTAGE, Vq=0` during alignment, verify the TIM1 update counter advances, verify the encoder is healthy, require measurable movement when commanding a positive electrical angle, return to angle zero, record direction/zero, then disable EN. Any failed predicate must set a motor fault and return nonzero.

- [ ] **Step 4: Replace the inline profile planner and map encoder health**

Use `motion_profile_t` for PP mode. At each 1 kHz tick, consume the most recent encoder sample, start the next sample, stop and latch `MOTOR_FAULT_ENCODER` after the stale threshold, and use the profile output as the velocity command. Cache electrical angle once per FOC ISR.

- [ ] **Step 5: Make `main()` stop on failed motor initialization/alignment**

Do not start CANopen as an apparently healthy drive after failed FOC initialization. Print the exact motor fault, leave EN low, but keep the main diagnostic/CANopen loop alive so the fault can be read.

- [ ] **Step 6: Verify RED becomes GREEN**

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

Expected: scheduling source check passes, native tests pass, firmware exits 0.

- [ ] **Step 7: Commit**

```powershell
git add src/board.c src/config.h src/motor.c src/motor.h src/main.c tools/run_native_tests.ps1
git commit -m "fix: run and validate the 20kHz FOC loop"
```

### Task 7: Integrate the tested CiA402 state machine and fault mapping

**Files:**
- Modify: `src/cia402.c`
- Modify: `src/cia402.h`
- Modify: `src/motor.h`
- Modify: `src/main.c`

- [ ] **Step 1: Add failing integration assertions**

Extend native tests to assert invalid modes are rejected by a small `cia402_mode_supported(int8_t mode)` helper for `0, 2, 4`, and accepted for `1, 3, 6`.  
Expected: native compile fails because the helper is missing.

- [ ] **Step 2: Replace duplicate state logic with `cia402_sm`**

`cia402_process()` passes `6040h` and motor-fault presence into `cia402_sm_step()`, enables the motor only when `cia402_sm_operation_enabled()` is true, and obtains base status bits from `cia402_sm_statusword()`. Add target-reached bits afterward.

- [ ] **Step 3: Map precise faults and modes**

Map overcurrent to `603Fh=2310h`, encoder failure to `7300h`, control timing/init failure to `FF01h`. Clear both `603Fh` and `1001h` on a valid fault reset edge. For invalid operation mode, set display to zero, command stop, and never issue a motion target.

- [ ] **Step 4: Verify native tests and full firmware build**

Run both verification commands.  
Expected: all native tests pass; firmware build exits 0 with no application warnings.

- [ ] **Step 5: Commit**

```powershell
git add src/cia402.c src/cia402.h src/cia402_sm.c src/cia402_sm.h src/main.c src/motor.h test/native/test_main.c
git commit -m "refactor: integrate tested CiA402 drive states"
```

### Task 8: Add watchdog, operator diagnostics, and CAN acceptance tooling

**Files:**
- Modify: `src/board.h`
- Modify: `src/board.c`
- Modify: `src/main.c`
- Modify: `tools/drive_test.py`
- Modify: `README.md`

- [ ] **Step 1: Add a failing CLI validation test**

Run `python tools/drive_test.py --help` and inspect that the help does not yet describe the exact enable/disable/status workflow and PcanView raw SDO frames. Treat the missing documented behavior as RED.

- [ ] **Step 2: Add independent watchdog interfaces**

Implement `board_watchdog_init()` for approximately 1.5 s and `board_watchdog_refresh()`. Start it only after board/motor/CANopen initialization. Refresh only after the main loop has processed CANopen and CiA402 and the 1 kHz/control counters continue changing; a stalled control system must reset instead of holding torque indefinitely.

- [ ] **Step 3: Improve diagnostics**

The once-per-second line must include statusword, controlword, mode, target velocity, position, measured velocity, Vq, U-phase current, motor fault, TIM1 update count, encoder error count, and CAN error status.

- [ ] **Step 4: Make the Python drive test deterministic**

The `enable` command writes `6040h=0006h`, waits for Ready To Switch On, writes `0007h`, waits for Switched On, writes `000Fh`, and waits for Operation Enabled. `disable` writes zero. `vel` writes mode 3 and a small target only after successful enable. All waits have timeouts and print statusword/error code on failure.

- [ ] **Step 5: Document PcanView/raw CAN procedure and serial upload**

Document the repository-local serial upload command, CAN 1 Mbps, node ID 1, raw SDO request/response COB-IDs (`0x601`/`0x581`), and controlword sequence. State that PEAK PcanView requires compatible hardware and that the UCC-T01 vendor tool may be required.

- [ ] **Step 6: Run verification**

Run:

```powershell
python -m py_compile tools/drive_test.py
python tools/drive_test.py --help
powershell -ExecutionPolicy Bypass -File tools/run_native_tests.ps1
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

Expected: all commands exit 0.

- [ ] **Step 7: Commit**

```powershell
git add src/board.c src/board.h src/main.c tools/drive_test.py README.md
git commit -m "feat: add drive watchdog and acceptance diagnostics"
```

### Task 9: Final static checks, requirements audit, and delivery build

**Files:**
- Modify if required: files implicated by verification failures
- Update: `docs/superpowers/plans/2026-08-18-canopen-foc-refactor.md`

- [ ] **Step 1: Run clean native tests**

Delete only `.pio/native-tests.exe`, rerun the host test runner, and require exit code 0.

- [ ] **Step 2: Run a clean firmware build**

Run:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target clean
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

Expected: exit code 0, no application warnings, and reported Flash/RAM sizes.

- [ ] **Step 3: Inspect the final diff and safety invariants**

Run `git diff main...HEAD --check`, search for `HAL_TIM_PWM_Start_IT(&htim1`, blocking `HAL_I2C_Mem_Read` outside initialization, dynamic allocation in application code, and missing timeout checks. Expected: no whitespace errors, no forbidden PWM start, and only the initial encoder probe remains blocking.

- [ ] **Step 4: Audit every design requirement**

Confirm the design sections for deterministic scheduling, non-blocking encoder runtime, standard CiA402 enable, exact fault mapping, default voltage-mode FOC, tests, serial upload, and GitHub handoff are each represented by code or documentation.

- [ ] **Step 5: Commit final verification metadata**

Mark completed checklist items in this plan and commit any final corrections:

```powershell
git add -A
git commit -m "docs: record CANopen FOC verification"
```

- [ ] **Step 6: Prepare local integration; defer GitHub push only while connectivity is unavailable**

Keep the completed branch and commits locally. Do not claim GitHub delivery until `git push -u origin refactor/canopen-foc-control` succeeds and the remote commit is verified.
