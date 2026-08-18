$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot '.pio'
$outputFile = Join-Path $outputDirectory 'native-tests.exe'

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

Push-Location $projectRoot
try {
    $motorSource = Get-Content -LiteralPath 'src/motor.c' -Raw
    if ($motorSource -match 'HAL_TIM_PWM_Start_IT\s*\(\s*&htim1') {
        throw 'motor.c must not use HAL_TIM_PWM_Start_IT for TIM1 PWM channels'
    }
    if ($motorSource -notmatch 'HAL_TIM_Base_Start_IT\s*\(\s*&htim1\s*\)') {
        throw 'motor.c must explicitly start the TIM1 update interrupt'
    }
    $currentLoopStart = $motorSource.IndexOf('void motor_current_loop_isr')
    $alignmentBranch = $motorSource.IndexOf('if (s_aligning != 0u)', $currentLoopStart)
    $overcurrentCheck = $motorSource.IndexOf('fabsf(s_iu) > OVERCURRENT_TRIP_A', $currentLoopStart)
    if ($currentLoopStart -lt 0 -or $alignmentBranch -lt 0 -or $overcurrentCheck -lt $currentLoopStart -or $overcurrentCheck -gt $alignmentBranch) {
        throw 'motor current ISR must enforce overcurrent protection before alignment output'
    }
    $alignmentEnd = $motorSource.IndexOf('if (!s_enabled', $alignmentBranch)
    if ($alignmentEnd -lt 0 -or $motorSource.Substring($alignmentBranch, $alignmentEnd - $alignmentBranch) -match 'voltage_limiter_step') {
        throw 'FOC alignment must use the validated fixed alignment voltage'
    }

    $driveTestSource = Get-Content -LiteralPath 'tools/drive_test.py' -Raw
    if ($driveTestSource -match 'network\.create_node\s*\(') {
        throw 'drive_test.py must not create a local CANopen node for the remote drive'
    }
    if ($driveTestSource -notmatch 'network\.add_node\s*\(') {
        throw 'drive_test.py must add the drive as a remote CANopen node'
    }

    $cia402Source = Get-Content -LiteralPath 'src/cia402.c' -Raw
    if ($cia402Source -notmatch 'if\s*\(\s*mode\s*!=\s*s_previous_mode\s*\)') {
        throw 'cia402.c must detect operation-mode transitions'
    }
    if ($cia402Source -notmatch '(?s)if\s*\(\s*mode\s*!=\s*s_previous_mode\s*\).*?motor_stop\s*\(\s*\)') {
        throw 'cia402.c must stop stale motion commands when the mode changes'
    }

    $boardSource = Get-Content -LiteralPath 'src/board.c' -Raw
    if ($boardSource -match '(?s)void\s+Error_Handler\s*\([^)]*\).*?__disable_irq\s*\(\s*\).*?HAL_Delay\s*\(') {
        throw 'Error_Handler must not call HAL_Delay after disabling SysTick interrupts'
    }

    $configSource = Get-Content -LiteralPath 'src/config.h' -Raw
    $voltageLimitMatch = [regex]::Match($configSource, '(?m)^#define\s+VOLTAGE_LIMIT_V\s+([0-9.]+)f')
    if (-not $voltageLimitMatch.Success -or [float]$voltageLimitMatch.Groups[1].Value -ne 3.0) {
        throw 'default voltage limit must be the validated 3.0 V ramped startup value'
    }
    $softCurrentMatch = [regex]::Match($configSource, '(?m)^#define\s+CURRENT_SOFT_LIMIT_A\s+([0-9.]+)f')
    if (-not $softCurrentMatch.Success -or [float]$softCurrentMatch.Groups[1].Value -lt 0.55 -or [float]$softCurrentMatch.Groups[1].Value -gt 0.65) {
        throw 'soft current limit must leave usable startup torque below the 0.8 A hard trip'
    }
    if ($configSource -notmatch '(?m)^#define\s+FOC_CURRENT_LOOP_ENABLE\s+0') {
        throw 'voltage FOC must remain the validated default for this single-phase sensor'
    }
    $currentLimitMatch = [regex]::Match($configSource, '(?m)^#define\s+CURRENT_LIMIT_A\s+([0-9.]+)f')
    if (-not $currentLimitMatch.Success -or [float]$currentLimitMatch.Groups[1].Value -lt 0.34 -or [float]$currentLimitMatch.Groups[1].Value -gt 0.36) {
        throw 'current-loop target must be limited to the validated 0.35 A startup value'
    }
    if ($configSource -notmatch '(?m)^#define\s+OVERCURRENT_CONFIRM_SAMPLES\s+8') {
        throw 'overcurrent protection must reject only sustained samples, not one startup spike'
    }

    if ($boardSource -notmatch 'ADC_INJECTED_SOFTWARE_START') {
        throw 'ADC injected conversion must use the validated software trigger'
    }
    if ($motorSource -notmatch 's_current_offset_raw' -or $motorSource -notmatch 'motor_calibrate_current_zero') {
        throw 'motor current sensing must calibrate its zero-current ADC offset before alignment'
    }
    if ($motorSource -notmatch '(?s)void\s+motor_current_loop_isr\s*\([^)]*\).*?adc_start_sample\s*\(') {
        throw 'TIM1 current ISR must continue to request injected ADC samples'
    }
    if ($motorSource -notmatch '(?s)#endif\s*\r?\n\s*vq\s*=\s*voltage_limiter_step') {
        throw 'current-loop output must pass through the common voltage limiter'
    }
    if ($motorSource -notmatch '(?s)s_aligning\s*=\s*0u;.*?s_iu\s*=\s*0\.0f;.*?s_iq\s*=\s*0\.0f;') {
        throw 'alignment shutdown must clear residual current feedback before enable'
    }

    & gcc -std=c11 -Wall -Wextra -Werror -Isrc `
        test/native/test_main.c `
        src/foc.c `
        src/fast_trig.c `
        src/voltage_limiter.c `
        src/pid.c `
        src/encoder_math.c `
        src/cia402_sm.c `
        src/motion_profile.c `
        -lm -o $outputFile
    if ($LASTEXITCODE -ne 0) {
        throw "Native test compilation failed with exit code $LASTEXITCODE"
    }

    & $outputFile
    if ($LASTEXITCODE -ne 0) {
        throw "Native tests failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
