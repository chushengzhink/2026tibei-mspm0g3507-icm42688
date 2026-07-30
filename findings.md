# Findings & Decisions

## Requirements
- Confirmed physical LF04 order is PA31, PA12, PB8, PA27 from left to right, encoded as B1/B2/B4/B8.
- Right-outer LF04 is physically PA27 and must be GPIO input with pull-up.
- Race line assist uses four digital sensors and remains bounded by 500 mm/s wheel targets.
- UART0 at 115200 8N1 exports a standard CSV on single-byte `D`; `C` clears only while stopped.
- CSV must include elapsed/lap time, target/actual center speed, left/right PWM, and line bits.
- Braking samples must remain in RAM until the three-cycle stopped criterion completes.

## Research Findings
- MSPM0G3507 PA27 is `IOMUX_PINCM60`; GPIO function is `IOMUX_PINCM60_PF_GPIOA_DIO27`.
- PA27 has alternate RTC/SPI/TIMG8/CAN functions, but `gpio_init(..., PA27, IN_UP)` explicitly selects GPIO input.
- Current race configuration already reaches the 20000/50000 hard cap through 14000 PID + 6000 feedforward.
- Previous telemetry was 32 bytes x 600 and stopped when `chassis_stop()` cancelled the running result.
- Current mission already measures key-edge-to-three-stopped-cycles elapsed time and displays it on OLED.
- Baseline host tests were 14/14 passing before implementation.
- `chassis_stop()` resets the velocity controller and disables idle capture, so the race app must stop once, then explicitly start idle capture while the mission waits for three stopped cycles.
- `line_follow_update()` already yields physical wheel-space bias (`left += correction`, `right -= correction`); a small race-specific state wrapper is needed for B0 timing, speed recovery, B6/B15 zeroing, and wheel headroom.
- `motor_velocity_measurement_t` is stubbed only by the idle-capture host test, so adding PWM fields has a contained compatibility surface.

## Technical Decisions
| Decision | Rationale |
|----------|-----------|
| Reuse line centroid and line_follow PD | Minimum change; existing module already implements weighted digital error and bounded correction |
| Combine line correction in wheel-speed space | Makes left/right steering intent explicit and allows dynamic wheel headroom limiting |
| Extend telemetry record to 40 bytes | Adds required diagnostics while retaining 600 samples; both ARM links fit and report 0 errors/warnings |
| Record absolute PWM magnitudes | Directly shows saturation despite opposite motor polarity signs |
| Add a focused `chassis_track_line_assist` module | Keeps stateful B0 recovery testable without coupling generic `line_follow` to one competition task |
| Require both wheel speeds below 20 mm/s | Matches the stated three-cycle stop gate; an average-speed test could finish while one wheel was still too fast |

## Verification Findings
- The extended empty CSV header is exactly 228 bytes including CRLF and contains 17 columns.
- Host coverage passes all 17 cases, including stage 0/1/2 compile variants and all B0-B15 sensor patterns.
- Default `project.uvprojx` and race `project_track.uvprojx` both link with `0 Error(s), 0 Warning(s)`.
- Final safety constants remain 6000 feedforward, 14000 race PID limit, and 20000 absolute duty cap; default self-test remains 11500 + 6000.
- Phase 15 final documentation audit found one stale README architecture-tree label (`P控制`); detailed README/WIRING/ROBOT_SETUP/HARDWARE_ACCEPTANCE text already describes the new `Kp=1.2, Kd=0.20` PD controller and direct-reversal confirmation.
- `chassis_telemetry_session_start()` calls `chassis_telemetry_clear()`, so every LF-only run starts with zero correction state and an empty buffer.
- `chassis_emergency_stop()` zeroes target speeds and cached PWM counts before the LF-only Center-stop snapshot; `chassis_capture_telemetry_now()` then records the same chassis timestamp with zero correction/target/PWM and replaces a same-timestamp periodic sample rather than duplicating it.
- The common mission/positive-distance `command_stop` branch must also clear the telemetry correction before its braking snapshot; otherwise the CSV would retain the last running infrared component after steering had stopped.

## LF04 Runtime Low-Level Investigation
- `gpio_init(..., IN_UP)` selected pull-up input but did not explicitly clear the GPIO DOE bit; DriverLib PINCM input configuration alone does not clear `GPIOx->DOE31_0`.
- The READY-to-RUN path contains no intentional LF04 pull-down or output configuration, and motor PWM uses PA28/PB20 rather than PA31/PA12/PB8/PA27.
- With 12 V disconnected, LF04 LEDs returned to normal when the race fault stopped PWM activity. Software protection will therefore explicitly clear DOE and reassert the four inputs at every sample before hardware investigation.
- `TRACK HW FAULT 05` with 12 V disconnected is a secondary `BUSY` result overwriting the underlying no-encoder/stall fault; fault ordering must be corrected.

## LF04 Runtime Protection Verification
- All GPIO input modes now clear DOE before applying input features; the production implementation is covered directly by a host test.
- Every LF04 read visits all four PA31/PA12/PB8/PA27 pull-up configurations, even when one call fails, and reports `io_fault` instead of a false `B15`.
- The final race Rebuild after these changes reports `Code=36772`, `0 Error(s), 0 Warning(s)`; the default-project final regression Rebuild remains pending.

## LF04 White-Baseline Regression
- Hardware reported white raw `RF` but stored baseline `W7`; therefore white normalized to `B8`.
- The observed left-to-right patterns are exact consequences of the bad baseline: `E^7=B9`, `D^7=BA`, `B^7=BC`, `7^7=B0`.
- This proves PA31/PA12/PB8/PA27 bit weights remain correct; the fix must remove adaptive polarity capture and gate READY on stable `RF`.
- Final verification is `18/18` host tests plus race/default Keil Rebuilds at `0 Error(s), 0 Warning(s)`; race/default code sizes are 36840/34324 bytes.

## Resources
- `code/chassis_track_app.c`, `code/chassis_track_mission.*`
- `code/line_sensor.*`, `code/line_follow.*`
- `code/chassis_telemetry.*`, `code/chassis.c`, `code/motor_velocity.*`
- `ml_libs/ml_board.h`, MSPM0G3507 device header

## Official Track Geometry and Failed-Lap Evidence
- The official H-problem PDF specifies a 16-20 mm black loop, 1500 mm straights, R500 semicircles, and a 50 mm long perpendicular A-line with the same width.
- The fixed LF04 centers are -40.25/-7.25/+7.25/+40.25 mm. The 33 mm inner-to-outer gaps create unavoidable 13-17 mm B0 blind zones, while a centered loop line covers the two inner sensors as B6.
- With the fixed 80.5 mm sensor span, the 50 mm A-line should not produce B15; normal race-usable patterns are B1/B2/B4/B6/B8.
- The uploaded race CSV contains 275 records: B0=181, B15=39, B9=16, plus B11/B13. Only about 32 nonzero samples are normal usable patterns.
- The lap completed in 27.34 s because line recovery held 200 mm/s for most of the route. Braking began near 200 mm/s and stopped about 41 mm beyond the encoder route, so the current run cannot calibrate the 100 mm/s stop lead.
- Telemetry wraps encoder/fused headings to +/-180 degrees and appends two different records at timestamp 62620; both are CSV representation defects, not fusion-state defects.

## Phase 8 Implementation Verification
- The race assist now accepts only B1/B2/B4/B6/B8, bridges B0 with a slew-limited remembered side, and latches the fifth consecutive impossible pattern.
- Remaining-distance speed limiting reaches the 100 mm/s final request before the dual gate; the assist layer treats the mission request as a hard upper bound.
- Telemetry is 44 bytes per record and exports 20 columns. Encoder/fused headings remain cumulative, duplicate timestamps coalesce, and the final zero-PWM snapshot replaces rather than duplicates the same timestamp.
- All 18 host groups pass. Both μVision projects are open interactively, so Phase 8 full Rebuild must be performed in those existing windows instead of starting another UV4 process.

## LF04-Only Diagnostic Evidence
- The latest 60-row CSV has only 8 samples accepted as usable, 40 B0 samples, and 10 impossible-pattern samples; fused heading rises from 0.02° to 29.05° in 5.84 s.
- A B4 sample immediately precedes the long B0 interval, so remembered-side recovery keeps commanding the same yaw direction and explains the large physical deviation.
- `CHASSIS_MODE_VELOCITY` always applies fused yaw-rate feedback when fusion is active. A true LF04-only steering test must use `CHASSIS_MODE_WHEEL_SPEED`; IMU may still be updated and recorded.
- The selected diagnostic workflow is boot-held Up, stopped-only 60/120/200 selection, 1000 mm automatic stop, and a 500 ms sustained-B0 lock-stop.
- The diagnostic controller will be a small host-testable state machine: three stopped-only speed levels, three-sample B6 start gate, 400 mm/s² distance-based speed profile, 1000 mm braking transition, three stopped cycles, and a 500 ms line-loss latch.
- Normal completion remains non-latched so another speed can be selected after CSV export and repositioning; B0 timeout, impossible pattern, GPIO, chassis, and Center-key faults remain emergency-latched.
- The existing line-assist already gives the required 8 mm/s-per-cycle correction slew and five-cycle impossible-pattern latch. In LF-only mode its left/right outputs can be sent directly through `CHASSIS_MODE_WHEEL_SPEED`; the mission heading command must not be called.
- The repeatable completion path must call `chassis_stop()` only once, start idle telemetry capture, and keep feeding measured wheel speeds into the line-test state machine until both wheels are below 20 mm/s for three cycles.
- Phase 9 final portable verification is `19/19` host groups. Both new/modified application sources compile directly with ARMCC, both μVision project XML files parse, and whitespace checks pass; only the full Rebuild in the already-open interactive μVision windows remains.

## LF04 Full-Pattern Hardware Correction
- Hardware confirms that, viewed from the vehicle front toward the rear, PA31/PA12/PB8/PA27 map left-to-right to B1/B2/B4/B8; GPIO order and steering sign are not to be swapped.
- The observed `LF SIGNAL FAULT` is caused by software rejecting B3/B5/B7/B9/B10/B11/B12/B13/B14 even though the real module can produce every B1-B14 pattern on the track.
- The selected policy accepts B1-B14 through the existing physical-position centroid, uses a +/-3 mm center deadband and three-cycle side reversal confirmation, and changes the diagnostic start gate to three valid centroid samples within +/-10 mm.
- B15 is a separate full-black ambiguity: preserve the last correction, ramp to at most 60 mm/s, recover after three valid samples, and latch `LF SIGNAL FAULT` only at 500 ms. B15 must not advance the B0 loss timer.

## Phase 11 User-Confirmed Grouped Control
- Phase 10 centroid and special-B15 decisions are superseded. The four inputs remain grouped as PA31/PA12 left and PB8/PA27 right.
- Every B1-B15 sample is valid: left-only group means left wheel slower/right wheel faster, right-only means the reverse, and both groups mean centered.
- Before Phase 11, production code and tests were reversed: at 120 mm/s a left-group sample expected left=180/right=60; the corrected result is left=93.6/right=146.4 with the new 0.22 P-only correction.
- Formal race keeps encoder/IMU route and finish logic, but their wheel-space steering bias may only reinforce the infrared direction; opposite assistance is discarded.
- LF-only mode remains boot-Up selected and direct-wheel controlled, but READY/COMPLETE permits Center start for any B0-B15 pattern.
- Baseline host suite passes the first ten cases then fails to link the line-control test because its source list omits `code/pid.c`.
- README, WIRING, ROBOT_SETUP, and the field acceptance checklist still contain the superseded 50%/60 mm/s no-PID behavior and the removed both-groups start gate; all four must be synchronized to avoid unsafe hardware validation.
- The four operating/acceptance documents are now synchronized: the boot-Up LF-only entry remains, only its pattern gate was removed, and the staged hardware check uses explicit left/right wheel-speed relationships.
- The complete portable suite now passes 18/18 after adding the missing PID source; `git diff --check` also passes.
- ARM Compiler 5 is available at `D:\Keil_v5\ARM\ARMCC\bin\armcc.exe`; the race project uses `__MSPM0G3507__`, `CHASSIS_TRACK_MISSION_BUILD=1`, and the user/code/ml_libs/MSPM0 SDK/CMSIS include paths.

## Phase 12 Unlimited LF04-Only Lap
- The current diagnostic default is 60/120/200 mm/s with a 1000 mm distance-triggered braking state.
- The application already treats Center during RUNNING as a latched emergency stop, so an unlimited lap needs no new stop interface.
- At 350 mm/s the 0.22 infrared bias is 77 mm/s, producing 273/427 mm/s wheel targets, below the existing 500 mm/s limit.
- The least disruptive compatible representation is `distance_mm=0` for unlimited operation while retaining the existing positive-distance braking path for callers/tests.
- One 6141.6 mm lap takes about 102.4 s at 60 mm/s, longer than the non-overwriting 60 s telemetry buffer; motion remains unaffected when the buffer fills.
- Focused host builds must include `-Itests/stubs -Icode -Iml_libs`; the line-control case also links `code/pid.c`.
- The post-change host suite passes 18/18, including unlimited travel beyond 6141.6 mm and the 350 mm/s 273/427 wheel targets.
- Both changed production sources compile directly with ARMCC under the race defines; both project XML files and whitespace checks pass.
- A user-owned `project_track.uvprojx` μVision window appeared before batch build, so both full Rebuilds remain an interactive handoff rather than launching a conflicting process.

## Phase 13 Four-Sensor Centroid Requirements
- The LF04 drawing confirms sensor-center intervals of 33/14.5/33 mm, giving positions `-40.25/-7.25/+7.25/+40.25 mm` about the module center.
- The normalized steering error is `-average(active_position_mm)/40.25`; left detections are positive and therefore slow the left wheel.
- B1-B15 are all valid centroid inputs. B6, B9, and B15 average to zero; only B0 is lost.
- B0 must keep the last valid normalized error at the current requested speed indefinitely. It remains a telemetry recovery state but cannot create a line-loss fault.
- If B0 occurs before any valid sample, the remembered error is zero: LF-only drives straight and formal racing may still use route assistance.
- The grouped fields and the 120 mm/s/300 ms fault path are confined to `line_sensor`, `chassis_track_line_control`, their tests, and the race application.
- Hardware feedback says the previous steering was insufficient. The selected conservative retune is `Kp=1.5`; with output limited to `±1`, outer-sensor correction remains capped while inner/intermediate centroid correction rises by 50%.
- After the `Kp=1.5` retune, all 18 host groups pass and all four affected production sources compile with ARM Compiler 5 under the race defines.
- Both Keil project XML files, affected-file trailing whitespace, `git diff --check`, and the unchanged speed/stall/PWM/steering safety constants pass final portable validation.
- Because an interactive `project_track.uvprojx` μVision window is open, no concurrent full build was started; both project Rebuilds remain the only outstanding Phase 13 verification.

## Phase 14 Outer-Single Steering Boost
- The source now contains the user's manual `Kp=1.8`, while centroid tests and operating documents still assume `Kp=1.5`; this explains the current `17/18` host baseline.
- B1/B8 already saturate the PID output at `±1`, so raising `Kp` cannot increase their steering. Their effective limits are the 22% speed ratio and 90 mm/s total steering cap.
- The confirmed behavior is a dedicated 35%/120 mm/s cap for current B1/B8 and for B0 only while the last valid pattern remains B1/B8. Any new valid pattern immediately selects its own normal or boosted mode.
- The implemented 400 mm/s boosted target `280/520` is proportionally scaled by the existing wheel limiter to approximately `269.23/500 mm/s`; 350 and 360 mm/s remain unscaled at `230/470` and `240/480`.
- The post-change portable suite passes all `18/18` groups, including dynamic 90/120 mm/s route arbitration and B1/B8 memory transitions.
- Four affected production sources pass ARM Compiler 5 checks; both project XML files, trailing whitespace, `git diff --check`, and unchanged chassis safety limits also pass.
- The already-open race μVision window prevents a parallel batch Rebuild. The remaining build acceptance is sequential user-triggered Rebuild of `project_track.uvprojx` and `project.uvprojx` at `0 Error(s), 0 Warning(s)`.

## Phase 15 Anti-Wobble PD and LF-Only CSV
- Hardware now completes a full LF04-only lap, but both 280 and 350 mm/s oscillate on straights and curves, with instability increasing with speed.
- The selected first retune is `Kp=1.2`, `Ki=0`, `Kd=0.20`; immediate B1/B8 35%/120 mm/s boost remains unchanged so the proven cornering authority is preserved.
- Pure LF04 steering remains infrared-only. UART export stays stopped-only; the existing 44-byte telemetry record's reserved byte will become signed `line_correction_mm_s`, producing 21 CSV columns without reducing the 600-record capacity.
- Current LF-only running already starts the 10 Hz RAM session and stopped fault state already permits `D`; the missing pieces are explicit Center-stop session finalization, a zero-correction final snapshot, and correction-field export.

## Phase 16 LF-Only Curve Memory Evidence
- The latest 350 mm/s LF-only CSV contains 181 records over 18.12 s; both curved sections spend about 79% of their samples in B0, with maximum continuous B0 runs of about 3.0 s and 3.7 s.
- During B0 with a commanded 120 mm/s correction, encoder speeds imply only about 84-85 mm/s realized wheel-space bias and roughly 45 deg/s yaw; the R500 geometry at the realized center speed needs about 77 mm/s.
- The dominant defect is therefore late curve acquisition followed by multi-second maximum correction, PWM saturation, and exit oversteer—not insufficient P gain.
- The selected fix is LF04-state curve memory only in LF-only mode: B1/B8 activate a signed hold bias, B15 clears it, and a confirmed opposite side clears or switches it. Formal racing retains its existing PD and route/IMU arbitration.
- In remembered B0, correction holds the 35%/120 mm/s outer boost for 600 ms, linearly tapers for 600 ms, then holds `min(speed*0.31, 110)`. At 350 mm/s this is 120, about 114.25 at 900 ms, and 108.5 mm/s from 1200 ms onward.
- The implemented line-only configuration is selected once at application initialization from the boot-Up mode flag; the formal-race configuration has `curve_memory_enabled=false`, so its route/IMU arbitration and B0 output remain unchanged.
- Focused regressions cover B1/B8 symmetry, 600/900/1200 ms boundaries, B1-B0-B2-B6, direct opposite confirmation, B15 clearing, initial B0, all five speed levels, and the 500 mm/s wheel limit.
- Portable verification passes: full host suite `18/18`, ARMCC production sources `4/4`, both Keil project XML files, `git diff --check`, and the unchanged 500 mm/s, PWM 20000, and 8-cycle stall limits.

## Phase 17 Premature Curve-Memory Exit Evidence
- The failed 350 mm/s session has 109 records over 10.92 s. Straight running is acceptable; B1 at 5.72 s correctly raises correction to +120 mm/s and fused heading from about 8 deg to 49 deg.
- B4 at 6.52-6.62 s clears positive curve memory after only about 0.8 s and 280 mm. Correction becomes -19/-16 mm/s, then B0 holds -16 for about 3.6 s while fused heading falls from about 48 deg to 23 deg, directly explaining the outside-track departure.
- The earlier completed lap shows first B1-to-exit and second B1-to-B15 spans of about 1.4-1.5 m. Therefore an opposite inner pattern at 280 mm is lateral recentering, not a curve-exit signal.
- At +120 mm/s commanded correction the realized yaw rate averages about 44.1 deg/s; R500 at the realized 325.6 mm/s needs about 37.3 deg/s. The existing 31%/110 mm/s long hold remains appropriate; gain and hold magnitude must not be increased.
- The user selected a requested-speed integral guard: curve exit cannot be accepted before 1200 mm of commanded travel, and encoder/IMU remain excluded from LF-only steering.
- The implemented guard saturates at 1200 mm, ignores B0 and opposite outer-single patterns for exit, and requires five accepted opposite non-outer samples after the distance gate. Repeated same-side B1/B8 does not reset travel; B15 still clears immediately.
- The failed sequence regression produces about +89/+92 mm/s for confirmed early B4 and returns to positive B0 boost/hold, eliminating the observed multi-second -16 mm/s reversal.
- Portable verification passes: host suite `18/18`, ARMCC sources `4/4`, both project XML files, `git diff --check`, and unchanged 500 mm/s, PWM 20000, and eight-cycle stall limits.
