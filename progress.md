# Progress Log

## Session: 2026-07-29

### Phase 1: Baseline & Interface Audit
- **Status:** complete
- Actions taken:
  - Read governing project documents and captured the dirty worktree.
  - Read implementation skills and recovered any prior planning session.
  - Confirmed PA27/PINCM60 GPIO capability and current telemetry/control architecture.
  - Audited all host-test stubs and the race/default Keil project membership.
- Files created/modified:
  - `task_plan.md`
  - `findings.md`
  - `progress.md`

### Phase 2: Sensor & Race Control
- **Status:** complete
- Actions taken:
  - Chose a dedicated, host-testable race line-assist wrapper around the existing PD controller.
  - Remapped the right-outer sensor to PA27/PINCM60, preserving PA31/PA12/PB8/PA27 left-to-right order.
  - Added weighted PD steering, B6/B15 zeroing, 100 ms loss decay, 200 mm/s recovery, and wheel headroom limiting.
  - Split straight and curve cruise targets with baseline/380/400 compile-time stages.
  - Connected the assist output to the race app while retaining encoder+IMU lap gates.
- Files created/modified:
  - `ml_libs/ml_board.h`
  - `code/chassis_track_line_assist.*`
  - `code/chassis_track_app.c`
  - `code/chassis_track_mission.*`
  - `user/project*.uvprojx`
  - `tests/chassis_track_line_assist_test.c`

### Phase 3: Telemetry & UART0 CSV
- **Status:** complete
- Actions taken:
  - Added absolute applied PWM counts to velocity and chassis status.
  - Expanded the RAM record to 40 bytes and appended session/speed/PWM/LF04 CSV columns.
  - Added race session start/finish metadata and immediate telemetry snapshots.
  - Continued 10 Hz passive capture through braking and added a final stopped snapshot.
  - Tightened the stop gate so both wheels, not their average, must remain below 20 mm/s for three cycles.
- Files created/modified:
  - `code/motor_velocity.*`
  - `code/chassis.*`
  - `code/chassis_telemetry.*`
  - `tests/chassis_telemetry_test.c`

### Phase 4: Tests, Docs & Builds
- **Status:** complete
- Actions taken:
  - Added all-pattern LF04 centroid tests and stage 1/stage 2 compile variants.
  - Added exact 40-byte record, 17-column CSV, UART busy/timeout, session timing, PWM, and braking regressions.
  - Updated README, wiring, setup, and hardware-acceptance documents for PA27 and UART0 CSV.
  - Ran `git diff --check` successfully.
  - Rebuilt default and race Keil projects sequentially; both report `0 Error(s), 0 Warning(s)`.

### Phase 6: LF04 Runtime GPIO Protection
- **Status:** in_progress
- Actions taken:
  - Confirmed there is no intentional runtime pull-down and no LF04/motor-PWM pin overlap.
  - Identified that GPIO input initialization did not explicitly clear output enable.
  - Confirmed the no-12 V fault display can be overwritten by a secondary velocity-update `BUSY` status.
  - Added explicit DOE clearing for all GPIO input modes and LF04 reassertion/fault propagation.
  - Added a host-only forced include so `gpio_input_test` compiles the production `ml_gpio.c` without loading memory-mapped MCU headers.
  - Reordered the final race command-status guard to avoid evaluating an unset status after an LF04 IO fault.
  - Documented the alternating RUN diagnostics, LF GPIO lock-stop, and no-12 V `TRACK STALL` acceptance path.
  - Changed LF04 reassertion to visit all four inputs even when one configuration call fails, while preserving the first error for lock-stop handling.

### Phase 7: LF04 Fixed Polarity & White-Ready Gate
- **Status:** complete
- Actions taken:
  - Reproduced `B9/BA/BC/B0` algebraically from current `RF` and stale `W7`.
  - Confirmed no pin-order change is needed; the adaptive startup baseline is the fault.
  - Fixed normalization to active-low black with constant `WF` and retained calibration API as validation-only.
  - Added a ten-sample white stability guard and combined it with the IMU completion gate before READY.
  - Updated OLED waiting diagnostics and LF04 setup/acceptance documentation.
  - Re-ran the complete host suite after fixed-polarity and gate changes: `18/18 passed`.
  - Compiled `line_sensor.c` and `chassis_track_app.c` directly with ARMCC successfully; only SDK/project-wide remarks appeared, with no source error.
  - Rebuilt the final race and default Keil projects: race `Code=36840`, default `Code=34324`, both `0 Error(s), 0 Warning(s)`.

### Phase 8: Track-Geometry Recovery & CSV Repair
- **Status:** in_progress
- Actions taken:
  - Parsed the official H-problem PDF and confirmed the 16-20 mm loop line and 50 mm A-line dimensions.
  - Matched those dimensions against the fixed 33/14.5/33 mm LF04 spacing and identified the unavoidable B0 blind zones.
  - Parsed the failed 27.34 s race CSV and separated legitimate B0 blind/loss states from physically implausible B15 and disconnected patterns.
  - Confirmed the requested implementation scope and preserved all existing motor, encoder, IMU, emergency-stop, stall, and PWM limits.
  - Implemented geometry-aware B1/B2/B4/B6/B8 filtering, bounded last-side B0 search, correction slew, opposite-side confirmation, and a five-cycle signal-fault latch.
  - Replaced time-only final deceleration with a remaining-distance speed ceiling and made lower speed requests authoritative over line recovery.
  - Reworked and passed focused host tests for the fixed blind-zone sequence, impossible patterns, recovery limits, final speed cap, and nominal 20 s lap budget.
  - Integrated the sticky LF signal fault into the race app, including immediate emergency stop, zero-PWM final snapshot, and dedicated OLED text.
  - Added rate-limited `BUSY` replies for race `D/C` commands without exporting or clearing RAM while running or braking.
  - Expanded telemetry to 44-byte records and 20 CSV columns with cumulative encoder/fused headings, duplicate-timestamp coalescing, and line-state flags.
  - Updated README, wiring, setup, and field acceptance documents for official line geometry, fixed LF04 blind zones, impossible patterns, final speed control, CSV purity, and SSCOM settings.
  - Ran all focused tests and the complete host suite successfully; ARMCC compiled the five Phase 8 source files successfully and `git diff --check` passed.
  - Detected both default and race μVision windows open, so did not start a conflicting UV4 process. Full Rebuild remains for the existing interactive windows.

## Test Results
| Test | Input | Expected | Actual | Status |
|------|-------|----------|--------|--------|
| Pre-change host suite | `tests/run_host_tests.ps1` | 14/14 | 14/14 | PASS |
| Final host suite | `tests/run_host_tests.ps1` | 17/17 | 17/17 | PASS |
| LF04 GPIO protection host suite | `tests/run_host_tests.ps1` | 18/18 | 18/18 | PASS |
| Default Keil Rebuild | `user/project.uvprojx` | 0 errors, 0 warnings | 0 errors, 0 warnings | PASS |
| Race Keil Rebuild | `user/project_track.uvprojx` | 0 errors, 0 warnings | 0 errors, 0 warnings | PASS |
| Race Rebuild after initial GPIO protection patch | interactive μVision | 0 errors, 0 warnings | 0 errors, 0 warnings; Code=36744 | PASS, superseded by final all-four reassertion edit |
| Final race Rebuild after all-four reassertion | interactive μVision | 0 errors, 0 warnings | 0 errors, 0 warnings; Code=36772 | PASS |
| Final race Rebuild with fixed `WF` gate | interactive μVision | 0 errors, 0 warnings | 0 errors, 0 warnings; Code=36840 | PASS |
| Final default Rebuild with fixed `WF` API | interactive μVision | 0 errors, 0 warnings | 0 errors, 0 warnings; Code=34324 | PASS |
| Phase 8 focused line assist | GCC C99 | pass | `chassis track line assist tests passed` | PASS |
| Phase 8 focused mission | GCC C99 | pass | `chassis track mission tests passed` | PASS |
| Phase 8 focused telemetry/UART | GCC C99 | pass | `chassis telemetry tests passed` | PASS |
| Phase 8 complete host suite | `tests/run_host_tests.ps1` | 18/18 | 18/18 | PASS |
| Phase 8 ARMCC source compile | 5 affected `.c` files | 5/5 | 5/5 | PASS |
| Phase 8 whitespace validation | `git diff --check` | pass | pass | PASS |
| Phase 8 default/race full Rebuild | existing interactive μVision windows | 0 errors, 0 warnings | pending user Rebuild | PENDING |

## Error Log
| Timestamp | Error | Attempt | Resolution |
|-----------|-------|---------|------------|
| 2026-07-29 | None | 1 | N/A |
| 2026-07-29 | Host suite stopped at `line_sensor_test` compilation after PA27 assertion | 1 | Running the single GCC command to obtain the complete diagnostic |
| 2026-07-29 | Test stub still mapped C8 to PB10 | 1 | Added PA27/PINCM60 constants and changed the stub mapping |
| 2026-07-29 | Stop gate averaged wheel speeds | 1 | Changed to independent left/right thresholds and added a regression |
| 2026-07-29 | `gpio_input_test` loaded production `ml_gpio.h` from `ml_libs` and could not find the MCU SDK header | 1 | Force-include the minimal test `ml_gpio.h` with the production include guard for this test only |
| 2026-07-29 | Production `gpio_get()` required `DIN31_0`, missing from the host GPIO register stub | 1 | Added the read-only input register field to the test stub |
| 2026-07-29 | ARMCC temp-build command was rejected because it recursively removed a computed temporary directory | 1 | Use the ignored `user/Objects` directory for fixed single-file check outputs without dynamic deletion |
| 2026-07-29 | Phase 8 line-assist compile failed after misplaced local declarations | 1 | Moved pattern and side locals from the wheel-limit helper to the update function |
| 2026-07-29 | Phase 8 line-assist assertions retained pre-slew PD values | 1 | Corrected expected inner correction and B0 search ramp values; mission tests already passed |
| 2026-07-29 | Combined Phase 8 documentation patch failed because one `ROBOT_SETUP.md` telemetry line differed from the expected context | 1 | Re-read the exact section and split the update into smaller file-specific patches |
| 2026-07-29 | Final audit `rg` call treated PowerShell-style wildcard paths as invalid Windows paths | 1 | Re-run against `user/Objects` with `-g` filters |
| 2026-07-29 | Safety audit repeated the same invalid literal wildcard pattern for source files | 1 | Re-run against directories with `-g 'ml_motor_driver.*' -g 'motor_velocity.*'` |
| 2026-07-29 | Phase 8 plan update duplicated four checklist items as both pending and complete | 1 | Removed the stale pending copies and kept the accurate completion state |
| 2026-07-29 | Final line-number lookup used `tests/chassis_*test.c` as a literal path | 1 | Re-run against the tests directory with an `-g` filter |

## 5-Question Reboot Check
| Question | Answer |
|----------|--------|
| Where am I? | Phase 8 implementation, host verification, docs, and ARMCC source checks complete; interactive full Rebuild pending |
| Where am I going? | Rebuild both open μVision projects, then staged LF04 and race hardware acceptance |
| What's the goal? | Implement safe LF04-assisted race mode with UART0 CSV diagnostics |
| What have I learned? | See findings.md |
| What have I done? | Implemented PA27 line assist, extended UART0 telemetry, tests, docs, and both Keil builds |

## Session: 2026-07-30

### Phase 9: LF04-Only Diagnostic Mode
- **Status:** in_progress
- Actions taken:
  - Re-read project constraints and implementation/file-planning skills.
  - Preserved the dirty worktree and recovered the existing Phase 8 plan state.
  - Parsed the latest CSV: 60 rows, 8 usable, 40 B0, 10 invalid, and 29.05° final fused heading after 5.84 s.
  - Confirmed velocity mode adds fused yaw-rate feedback, so LF04-only steering requires continuous direct wheel-speed updates.
  - Locked the user-selected interface: boot-held Up, stopped-only 60/120/200, 1000 mm stop, and 500 ms B0 fault.
  - Added the host-testable line-only diagnostic state machine and continuous direct-wheel update API.
  - Passed focused diagnostic and chassis-motion tests, including no IMU yaw correction in direct wheel mode and preserved 160 ms stall accumulation.
  - Planning bookkeeping patches failed twice due to mixed or misplaced cross-file context; switched to exact single-file patches without affecting source changes.
  - Resumed from the partial application integration, re-read all governing documents and confirmed both existing interactive Keil windows remain open; no additional UV4 build process will be started.
  - Completed the LF-only poll-loop integration: stopped-only speed keys, released-Up plus B6 start gate, direct wheel commands, 500 ms B0 lock-stop, one-shot braking, three-cycle stop confirmation, and repeatable finished-page alignment.
  - Froze the displayed/exported diagnostic elapsed time at the confirmed stop timestamp and added a regression.
  - Added the diagnostic module to both μVision projects and the 19th host-test group.
  - Updated README, setup, wiring, and hardware acceptance instructions for boot-held Up, B6 gate, 60/120/200 selection, 1000 mm stop, fault behavior, and staged straight/curve validation.
  - Ran the complete host suite successfully: `19/19 passed`; `git diff --check` also passed.
  - Compiled `chassis_track_line_test.c` and `chassis_track_app.c` directly with ARMCC for the race define; both succeeded with no diagnostics.
  - Removed an extra start-edge state-machine update so the first commanded step remains exactly 8 mm/s per 20 ms, then recompiled the application with ARMCC successfully.
  - Final project XML, tracked/untracked trailing-whitespace, and `git diff --check` validation all pass; both interactive μVision windows are still open for the required user-triggered full Rebuild.

### Phase 10: LF04 Full-Pattern Tracking
- **Status:** in_progress
- Actions taken:
  - Re-read project constraints and implementation/file-planning skills, preserved the dirty worktree, and confirmed both interactive μVision windows remain open.
  - Confirmed the hardware mapping B1/B2/B4/B8 from vehicle-front view and isolated the false stop to the B3-B14 rejection policy rather than GPIO order.
  - Locked the requested B1-B14 centroid, centered-start, and B15 60 mm/s / 500 ms ambiguity behavior.

### Phase 11: Infrared-Priority Steering Correction
- **Status:** in_progress
- Actions taken:
  - Re-read the governing project documents and implementation skills, recovered the existing file plan, and preserved the dirty worktree.
  - Recorded that the earlier centroid/B15 policy is superseded by the confirmed two-group interpretation.
  - Reproduced the reversed wheel-bias expectations in the current source/tests.
  - Ran the baseline host suite: tests 1-10 passed; test 11 failed to link because `code/pid.c` is missing from its test case.
  - Corrected the grouped control sign, installed the P-only 0.22/90 mm/s defaults, and added sign-constrained encoder/IMU wheel-space assistance.
  - Added `code/pid.c` to the host line-control test dependency list.
  - Removed the LF-only straight/both-groups start gate from its state, API, application integration, tests, and OLED prompts.
  - Changed normal and diagnostic READY text so firmware no longer claims to recognize A from LF04 state.
  - A combined HARDWARE_ACCEPTANCE patch failed on one wording mismatch; no part of that patch was applied, so the next attempt will use exact focused context.
  - Updated README, WIRING, ROBOT_SETUP, and HARDWARE_ACCEPTANCE for the preserved boot-Up LF-only entry, unconditional Center start, corrected wheel directions, P-only parameters, and infrared-priority arbitration.
  - Ran the complete host suite successfully: `18/18 passed`; `git diff --check` passed with only existing line-ending notices.
  - Compiled `chassis_track_line_control.c`, `chassis_track_line_test.c`, `chassis_track_app.c`, and `chassis_track_mission.c` directly with ARMCC: `4/4 passed`.
  - Parsed both Keil project XML files and re-audited the 500 mm/s wheel, 8-cycle stall, race PID, and 20000 PWM safety limits; all remain intact.
  - Confirmed one interactive μVision window is open on `project_track.uvprojx`; no additional UV4 process was started.
