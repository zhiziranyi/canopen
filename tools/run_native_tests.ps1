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

    & gcc -std=c11 -Wall -Wextra -Werror -Isrc `
        test/native/test_main.c `
        src/foc.c `
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
