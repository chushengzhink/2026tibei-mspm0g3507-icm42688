param()

$ErrorActionPreference = 'Stop'

$gccCommand = Get-Command gcc -ErrorAction SilentlyContinue
if ($null -eq $gccCommand) {
    [Console]::Error.WriteLine(
        'gcc was not found on PATH. Install MinGW/GCC before running host tests.')
    exit 2
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path ([System.IO.Path]::GetTempPath()) (
    'mspm0g3507-host-tests-' + [Guid]::NewGuid().ToString('N'))
$commonArguments = @(
    '-std=c99',
    '-Wall',
    '-Wextra',
    '-Werror',
    '-Itests/stubs',
    '-Icode',
    '-Iml_libs'
)
$cases = @(
    [pscustomobject]@{
        Name = 'gpio_input_test'
        Arguments = @('-include', 'tests/stubs/ml_gpio.h')
        Sources = @('tests/gpio_input_test.c', 'ml_libs/ml_gpio.c')
    },
    [pscustomobject]@{
        Name = 'quadrature_test'
        Sources = @('tests/quadrature_test.c', 'ml_libs/ml_quadrature.c')
    },
    [pscustomobject]@{
        Name = 'chassis_odometry_test'
        Sources = @('tests/chassis_odometry_test.c', 'code/chassis_odometry.c')
    },
    [pscustomobject]@{
        Name = 'chassis_heading_fusion_test'
        Sources = @(
            'tests/chassis_heading_fusion_test.c',
            'code/chassis_heading_fusion.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_motion_test'
        Sources = @(
            'tests/chassis_motion_test.c',
            'code/chassis_motion.c',
            'code/chassis_heading_fusion.c',
            'code/chassis_odometry.c',
            'code/chassis_config.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_key_test'
        Sources = @('tests/chassis_key_test.c', 'code/chassis_key.c')
    },
    [pscustomobject]@{
        Name = 'chassis_self_test_view_test'
        Sources = @(
            'tests/chassis_self_test_view_test.c',
            'code/chassis_self_test_view.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_telemetry_test'
        Sources = @(
            'tests/chassis_telemetry_test.c',
            'code/chassis_telemetry.c',
            'code/chassis_telemetry_uart.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_idle_capture_test'
        Sources = @(
            'tests/chassis_idle_capture_test.c',
            'code/chassis.c',
            'code/chassis_motion.c',
            'code/chassis_heading_fusion.c',
            'code/chassis_odometry.c',
            'code/chassis_config.c',
            'code/chassis_telemetry.c'
        )
    },
    [pscustomobject]@{
        Name = 'line_sensor_test'
        Sources = @('tests/line_sensor_test.c', 'code/line_sensor.c')
    },
    [pscustomobject]@{
        Name = 'chassis_track_line_control_test'
        Sources = @(
            'tests/chassis_track_line_control_test.c',
            'code/chassis_track_line_control.c',
            'code/pid.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_track_line_test_test'
        Sources = @(
            'tests/chassis_track_line_test_test.c',
            'code/chassis_track_line_test.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_track_mission_test'
        Sources = @(
            'tests/chassis_track_mission_test.c',
            'code/chassis_track_mission.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_track_mission_stage1_test'
        Arguments = @('-DCHASSIS_TRACK_SPEED_STAGE=1')
        Sources = @(
            'tests/chassis_track_mission_test.c',
            'code/chassis_track_mission.c'
        )
    },
    [pscustomobject]@{
        Name = 'chassis_track_mission_stage2_test'
        Arguments = @('-DCHASSIS_TRACK_SPEED_STAGE=2')
        Sources = @(
            'tests/chassis_track_mission_test.c',
            'code/chassis_track_mission.c'
        )
    },
    [pscustomobject]@{
        Name = 'h456_mission_test'
        Sources = @(
            'tests/h456_mission_test.c',
            'code/h456_mission.c'
        )
    },
    [pscustomobject]@{
        Name = 'h456_telemetry_test'
        Sources = @(
            'tests/h456_telemetry_test.c',
            'code/h456_telemetry.c'
        )
    },
    [pscustomobject]@{
        Name = 'h456_app_test'
        Arguments = @(
            '-iquote', 'tests/h456_app_stubs',
            '-include', 'tests/h456_app_stubs/prelude.h'
        )
        Sources = @(
            'tests/h456_app_test.c',
            'code/h456_app.c',
            'code/h456_mission.c',
            'code/h456_telemetry.c',
            'code/chassis_key.c'
        )
    },
    [pscustomobject]@{
        Name = 'pid_velocity_test'
        Sources = @(
            'tests/pid_velocity_test.c',
            'code/motor_velocity.c',
            'code/pid.c'
        )
    },
    [pscustomobject]@{
        Name = 'imu_attitude_test'
        Sources = @(
            'tests/imu_attitude_test.c',
            'code/imu_attitude.c',
            'ml_libs/FusionAhrs.c'
        )
    },
    [pscustomobject]@{
        Name = 'icm42688_service_test'
        Sources = @(
            'tests/icm42688_service_test.c',
            'code/icm42688_service.c',
            'code/imu_attitude.c',
            'ml_libs/FusionAhrs.c'
        )
    },
    [pscustomobject]@{
        Name = 'maix_ball_protocol_test'
        Sources = @(
            'tests/maix_ball_protocol_test.c',
            'code/maix_ball_protocol.c'
        )
    },
    [pscustomobject]@{
        Name = 'rds3230_test'
        Arguments = @('-iquote', 'tests/ball_stubs')
        Sources = @(
            'tests/rds3230_test.c',
            'code/rds3230.c'
        )
    },
    [pscustomobject]@{
        Name = 'ball_balance_test'
        Arguments = @(
            '-iquote', 'tests/ball_stubs',
            '-DBALL_BALANCE_ALLOW_SEQUENCE=1'
        )
        Sources = @(
            'tests/ball_balance_test.c',
            'code/ball_balance.c',
            'code/rds3230.c',
            'code/maix_ball_protocol.c'
        )
    },
    [pscustomobject]@{
        Name = 'ball_telemetry_test'
        Sources = @(
            'tests/ball_telemetry_test.c',
            'code/ball_telemetry.c'
        )
    },
    [pscustomobject]@{
        Name = 'ball_balance_app_formal_test'
        Arguments = @(
            '-iquote', 'tests/ball_app_stubs',
            '-DBALL_AUTO_CONTROL_MODE=0'
        )
        Sources = @(
            'tests/ball_balance_app_test.c',
            'code/ball_balance_app.c'
        )
    },
    [pscustomobject]@{
        Name = 'ball_app_telemetry_integration_test'
        Arguments = @(
            '-iquote', 'tests/ball_app_stubs',
            '-DBALL_AUTO_CONTROL_MODE=0'
        )
        Sources = @(
            'tests/ball_app_telemetry_integration_test.c',
            'code/ball_balance_app.c',
            'code/ball_telemetry.c'
        )
    },
    [pscustomobject]@{
        Name = 'ball_balance_app_center_test'
        Arguments = @(
            '-iquote', 'tests/ball_app_stubs',
            '-DBALL_AUTO_CONTROL_MODE=2'
        )
        Sources = @(
            'tests/ball_balance_app_test.c',
            'code/ball_balance_app.c'
        )
    },
    [pscustomobject]@{
        Name = 'ball_balance_app_speed_test'
        Arguments = @(
            '-iquote', 'tests/ball_app_stubs',
            '-DBALL_AUTO_CONTROL_MODE=1'
        )
        Sources = @(
            'tests/ball_balance_app_test.c',
            'code/ball_balance_app.c'
        )
    }
)

$results = New-Object System.Collections.Generic.List[object]
$locationPushed = $false

try {
    New-Item -ItemType Directory -Path $buildDirectory -Force | Out-Null
    Push-Location -LiteralPath $repoRoot
    $locationPushed = $true

    for ($index = 0; $index -lt $cases.Count; ++$index) {
        $case = $cases[$index]
        $executable = Join-Path $buildDirectory ($case.Name + '.exe')
        Write-Host ("[{0}/{1}] {2}" -f ($index + 1), $cases.Count, $case.Name)

        $caseArguments = @()
        if ($null -ne $case.Arguments) {
            $caseArguments = $case.Arguments
        }
        $compileArguments = $commonArguments + $caseArguments +
            $case.Sources + @('-lm', '-o', $executable)
        $compileOutput = & $gccCommand.Source @compileArguments 2>&1
        $compileExitCode = $LASTEXITCODE
        if ($compileOutput) {
            $compileOutput | ForEach-Object { Write-Host ("  {0}" -f $_) }
        }
        if ($compileExitCode -ne 0) {
            Write-Host '  FAIL (compile)' -ForegroundColor Red
            $results.Add([pscustomobject]@{ Name = $case.Name; Passed = $false })
            continue
        }

        $runOutput = & $executable 2>&1
        $runExitCode = $LASTEXITCODE
        if ($runOutput) {
            $runOutput | ForEach-Object { Write-Host ("  {0}" -f $_) }
        }
        if ($runExitCode -ne 0) {
            Write-Host ("  FAIL (exit code {0})" -f $runExitCode) -ForegroundColor Red
            $results.Add([pscustomobject]@{ Name = $case.Name; Passed = $false })
            continue
        }

        Write-Host '  PASS' -ForegroundColor Green
        $results.Add([pscustomobject]@{ Name = $case.Name; Passed = $true })
    }

    $passed = @($results | Where-Object { $_.Passed }).Count
    $failed = $cases.Count - $passed
    Write-Host ("Summary: {0}/{1} passed" -f $passed, $cases.Count)
    if ($failed -ne 0) {
        exit 1
    }
}
finally {
    if ($locationPushed) {
        Pop-Location
    }
    if (Test-Path -LiteralPath $buildDirectory) {
        Remove-Item -LiteralPath $buildDirectory -Recurse -Force
    }
}
