$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$outputDirectory = Join-Path $projectRoot '.pio'
$outputFile = Join-Path $outputDirectory 'native-tests.exe'

New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

Push-Location $projectRoot
try {
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
