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
    if (-not $voltageLimitMatch.Success -or [float]$voltageLimitMatch.Groups[1].Value -lt 2.5) {
        throw 'default voltage limit must provide enough startup torque (at least 2.5 V)'
    }

    & gcc -std=c11 -Wall -Wextra -Werror -Isrc `
        test/native/test_main.c `
        src/foc.c `
        src/fast_trig.c `
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
