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

### Phase 12: LF04-Only Unlimited Lap and 350 mm/s
- **Status:** in_progress
- Actions taken:
  - Re-read the project constraints and implementation/file-planning skills and preserved the dirty worktree.
  - Locked the user-selected five speeds and manual Center-key locked stop behavior.
  - Confirmed zero-distance sentinel compatibility, 350 mm/s wheel headroom, and the 60-second telemetry-buffer limitation.
  - Ran the pre-change full host suite successfully: `18/18 passed`.
  - Implemented the five-speed default, zero-distance unlimited path, OLED `NO LIMIT` text, and focused regressions.
  - First focused GCC command failed before source compilation because it omitted the existing `ml_libs` include path; the retry will reuse the test runner's exact flags.
  - The first error-log patch used stale table context and did not apply; switched to this exact Phase 12 progress entry.
  - Re-ran both focused tests with the correct includes; line-test and line-control regressions passed.
  - Updated README, WIRING, ROBOT_SETUP, and HARDWARE_ACCEPTANCE for five speeds, unlimited laps, locked manual stopping, safe CSV export, and the 60-second RAM limit.
  - Ran the complete post-change host suite successfully: `18/18 passed`.
  - Confirmed no UV4 process is running and recovered the race project's exact ARMCC defines/include paths for portable source checks.
  - Compiled `chassis_track_line_test.c` and `chassis_track_app.c` directly with ARMCC under the race defines; both passed without diagnostics.
  - Parsed both Keil project XML files and passed `git diff --check`; only existing LF/CRLF conversion notices were emitted.
  - The default Keil build was safely aborted before launch because a UV4 process appeared after the earlier check; no parallel builder was started.
  - Added the explicit 350 mm/s B0 regression and reran the full suite: `18/18 passed`.
  - Rechecked both project XML files, affected-file trailing whitespace, `git diff --check`, and all relevant speed/stall/line safety constants successfully.
  - Confirmed the newly opened interactive window is `project_track.uvprojx`; both required full Rebuilds are pending user action in μVision.

### Phase 13: Four-Sensor Centroid and Unlimited B0 Hold
- **Status:** in_progress
- Actions taken:
  - Re-read project constraints and implementation/file-planning skills and preserved the existing dirty worktree.
  - Confirmed the physical LF04 positions from the supplied drawing and the current grouped controller/lost-fault interface surface.
  - Locked user choices: both modes share the centroid controller, B0 holds error at full task speed indefinitely, startup B0 is centered, and PID starts P-only.
  - The first combined source patch did not apply because the application context field order differed; no source hunk was applied, so implementation will continue with exact file-specific patches.
  - Removed grouped sample fields, implemented the physical four-sensor centroid, retained PID history, and removed speed-reduction/timeout line-loss handling.
  - Removed the application `LF LOST STOP` path while preserving GPIO, chassis, Center-key, stall, and mission faults.
  - Replaced grouped controller regressions with all-16 centroid, per-sensor strength, route-priority, five remembered-state B0, startup B0, counter saturation, and PID-history tests.
  - Ran the first post-change complete host suite successfully: `18/18 passed`.
  - Incorporated new hardware feedback by increasing line `Kp` from 1.0 to 1.5 while preserving the 22% ratio, 90 mm/s bias cap, 500 mm/s wheel limit, and P-only structure.
  - Re-ran the complete host suite after the gain change: `18/18 passed`.
  - Confirmed obsolete grouped fields and line-loss timeout/fault identifiers are absent from production code and tests; remaining `LF LOST STOP` text only documents that the OLED must not show it.
  - Compiled `line_sensor.c`, `chassis_track_line_control.c`, `chassis_track_line_test.c`, and `chassis_track_app.c` directly with ARMCC under the race defines: `4/4 passed`.
  - Parsed both Keil project XML files, checked affected-file trailing whitespace, passed `git diff --check`, and re-audited the 500 mm/s wheel, 8-cycle stall, 20000 PWM, 90 mm/s correction, and 22% ratio limits.
  - Confirmed the interactive `project_track.uvprojx` window remains open; no parallel UV4 process was launched, so both full Rebuilds remain a user handoff.

### Phase 14: Outer-Single Boost and B0 Mode Memory
- **Status:** in_progress
- Actions taken:
  - Re-read the project constraints and implementation/file-planning skills, ran session catch-up, and preserved the dirty worktree.
  - Confirmed the user-selected 35%/120 mm/s boost and B1/B8-to-B0-only memory policy against the current controller.
  - Reproduced the pre-change host result: `17/18 passed`; only line-control expectations fail because source `Kp=1.8` no longer matches test `Kp=1.5`.
  - Added the dynamic outer-single configuration, last-valid-pattern memory, and focused transition/strength regressions.
  - The first focused GCC command failed before source compilation because `tests\build` does not exist; the retry will use an explicit executable in the system temporary directory like the host runner.
  - Re-ran the focused line-control test with strict GCC warnings from the system temporary directory; it passed.
  - Added an explicit non-outer 90 mm/s route-cap assertion and synchronized README, WIRING, ROBOT_SETUP, and HARDWARE_ACCEPTANCE with `Kp=1.8` and the dynamic boost behavior.
  - Ran the complete host suite after implementation: `18/18 passed`.
  - Confirmed the four operating documents no longer present `Kp=1.5` or the old B1/B8 22%/90 mm/s behavior as current instructions.
  - Added symmetric 90 mm/s limiting for the zero-centroid route-only branch and reran the complete host suite: `18/18 passed`.
  - Confirmed the interactive race μVision window remains open and compiled the four affected production sources directly with ARMCC: `4/4 passed`; no UV4 process was launched.
  - Parsed both Keil project XML files, passed affected-file trailing-whitespace and `git diff --check`, and re-audited the 500 mm/s wheel, 8-cycle stall, 20000 PWM, normal 90 mm/s, and outer 120 mm/s limits.
  - Portable Phase 14 verification is complete; only the two user-triggered full Rebuilds and hardware trial remain.

### Phase 15: Anti-Wobble PD and LF-Only CSV
- **Status:** in_progress
- Actions taken:
  - Re-read project constraints and implementation/file-planning skills, ran session catch-up, and confirmed a clean worktree with Phase 14 as the current baseline.
  - Locked the infrared-only PD, immediate outer boost, one-cycle direct-reversal confirmation, stopped-only export, and 21-column/no-extra-RAM telemetry decisions.
  - Ran the clean Phase 14 baseline host suite: `18/18 passed`.
  - Confirmed the telemetry record is exactly 44 bytes with a one-byte `reserved` field, and identified every 20-column/277-byte documentation and test expectation that must move to the 21-column header.
  - Implemented first-valid derivative initialization, `reverse_confirm_cycles=2`, centered/same-side immediate acceptance, and B0 confirmation of a pending direct reversal.
  - Updated focused controller regressions for steady inner correction, derivative release/centering pulses, glitch cancellation, second-sample confirmation, and B0 confirmation; the strict GCC test passes.
  - Replaced the telemetry record's reserved byte with signed `line_correction_mm_s`, added its setter/export column, and retained the 44-byte/600-record layout.
  - Integrated correction updates into both tracking modes and finalized LF-only sessions with a zero-correction snapshot on locked Center stop.
  - Updated and ran the focused telemetry/UART regression with strict warnings; signed saturation, 21-column export, stopped-only `D/C`, and same-timestamp replacement pass.
  - Calculated the exact 21-column empty CSV header length as 298 bytes including CRLF.
  - The first combined four-document patch failed on one ROBOT_SETUP UART wording mismatch and applied no hunks; documentation will be updated file by file.
  - Completed the four operating-document update; a final audit found and corrected one stale README architecture-tree `P控制` label.
  - Added a configuration boundary regression proving `reverse_confirm_cycles=0` is rejected at initialization.
  - Re-ran the complete host suite after the cleanup and boundary test: `18/18 passed`.
  - Confirmed the user-owned `project_track.uvprojx` μVision window is open and did not launch a competing UV4 build.
  - Compiled `chassis_track_line_control.c`, `chassis_telemetry.c`, `chassis_telemetry_uart.c`, and `chassis_track_app.c` directly with ARMCC under the race project's actual defines/includes: `4/4 passed` with no diagnostics.
  - Parsed both Keil project XML files, passed `git diff --check`, and re-audited the 500 mm/s wheel, 90/120 mm/s correction, 22%/35% ratio, and 8-cycle stall constants.
  - Added exact `+120 mm/s` correction storage coverage; the final complete host suite remains `18/18 passed`.
  - Cleared line-correction telemetry in the shared braking entry so formal-race and positive-distance stop snapshots report zero once steering is no longer applied; recompiled the application with ARMCC successfully.

### Phase 16: LF-Only Curve Memory and Smooth B0 Correction
- **Status:** in_progress
- Actions taken:
  - Re-read the project constraints and implementation/file-planning guidance, ran `git status --short`, and preserved all Phase 15 working-tree changes.
  - Locked the implementation boundary: curve memory and B0 taper apply only to boot-Up LF-only mode; formal race configuration and route/IMU arbitration remain unchanged.
  - Recorded the 181-row CSV evidence and the selected 600 ms full-boost, 600 ms taper, 31%/110 mm/s long-hold behavior without changing the 350 mm/s center speed.
  - Added a second exported LF-only line-control configuration and selected it only when boot-Up mode is active; formal racing continues to use the default configuration with curve memory disabled.
  - Implemented signed B1/B8 curve memory, B15/opposite-side clearing, same-side hold plus PD residual, exact B6/B9 hold, and the 600/600 ms B0 interpolation.
  - Ensured the first pending direct-reversal cycle outputs zero without leaking the old remembered curve; B0 can still complete the existing second-cycle confirmation.
  - Added focused regressions for timing boundaries, state transitions, symmetry, five speeds, long B0, and the wheel limit; the focused test and full host suite pass, with the latter at `18/18`.
  - Updated README, WIRING, ROBOT_SETUP, and HARDWARE_ACCEPTANCE with the two-mode distinction, timing curve, UART CSV expectations, and field acceptance targets.
  - Compiled `chassis_track_line_control.c`, `chassis_track_app.c`, `chassis_telemetry.c`, and `chassis_telemetry_uart.c` with ARM Compiler 5 under the race project defines/includes: `4/4 passed`.
  - Parsed both Keil project XML files, passed `git diff --check`, and re-audited the 500 mm/s wheel, 20000 PWM, 8-cycle stall, and curve-memory constants.
  - Confirmed the interactive `project_track.uvprojx` μVision process remains open; no competing UV4 build was launched, so both full Rebuilds remain a user handoff.

### Phase 17: LF-Only Curve Exit Travel Guard
- **Status:** in_progress
- Actions taken:
  - Re-read project constraints and both implementation/file-planning skills, ran session catch-up and `git status --short`, and preserved all existing Phase 15/16 changes.
  - Parsed the new 109-row CSV and compared it with the earlier completed-lap CSV; isolated premature B4-triggered memory clearing as the direct cause of the departure.
  - Locked the user-selected requested-speed integral guard: 1200 mm minimum curve travel plus five accepted opposite cycles, with no encoder/IMU steering input.
  - Added saturating requested-travel accumulation, five-cycle accepted-opposite exit confirmation, immediate B15 clearing, and reset/new-session state cleanup without changing the public update signature.
  - Kept early opposite inner patterns inside the active curve as PD residual; the failed sequence now produces about +89/+92 mm/s and its following B0 remains positive instead of holding -16 mm/s.
  - Expanded focused regressions for the failed 280 mm sequence, 1200 mm boundary, B0 interruption, B15, same-side B1 travel preservation, left/right symmetry, and all five speeds; the strict GCC test passes.
  - Added the opposite-outer exclusion and formal-race-disabled regressions, then ran the complete host suite successfully at `18/18 passed`.
  - Updated README, WIRING, ROBOT_SETUP, and HARDWARE_ACCEPTANCE with the 1200 mm/five-cycle exit guard, early opposite residual behavior, and CSV acceptance checks without changing the 21-column format.
  - Compiled the controller, application, telemetry, and telemetry-UART sources with ARM Compiler 5 under race defines/includes: `4/4 passed`.
  - Parsed both Keil project XML files, passed `git diff --check`, and re-audited the 500 mm/s wheel, PWM 20000, eight-cycle stall, and 1200 mm/five-cycle guard constants.
  - Confirmed the existing interactive `project_track.uvprojx` μVision process is still open; no competing UV4 build was launched.

### Phase 18: Formal IMU + LF04 + Encoder Arbitration
- **Status:** in_progress
- Actions taken:
  - Re-read project constraints and the implementation/file-planning guidance, ran session catch-up and `git status --short`, and preserved all existing Phase 17 working-tree changes.
  - Read and visually checked the four-page H-problem PDF; confirmed the 20 s, 20 mm, R500, 1500 mm, infrared-only line-sensor, and safety-relevant requirements.
  - Parsed the duplicated 398-row UART paste into three monotonic segments and selected the 187-row first session as authoritative.
  - Reconstructed route progress/expected heading and confirmed that formal infrared priority prevents already-computed encoder+IMU damping from affecting the final steering command.
  - Locked the implementation boundary: formal race receives signed confidence/budget arbitration; boot-Up LF-only behavior, speed stages, lower yaw-rate loop, motor limits, stall protection, and emergency stop remain unchanged.
  - Added split mission outputs, formal-only steering-budget arbitration, 120 ms B0 decay, sustained-B1 taper, heading-conflict attenuation, and 52-byte telemetry plumbing.
  - Ran the first integrated host suite: `14/18 passed`; all core sources compile and all unchanged LF-only tests pass. Expected failures are confined to the old 44-byte/21-column telemetry strings and old 350-degree finish-gate assertions.
  - Completed the original confidence-budget implementation, 27-column telemetry, finish-window tests, and host suite at `18/18`, then observed a race Rebuild SRAM overflow of `0xC10` bytes.
  - Replaced the 600-by-52-byte RAM array with a 44-byte compact internal record while keeping the 52-byte public structure, 27-column export, and 600-record capacity; host tests and ARMCC passed.
  - Parsed the subsequent failed hardware CSV: the car departed before finishing the first semicircle because live B8 `-120 mm/s` was reversed into `+65..+84 mm/s` by `+77/+37` route/heading bias.
  - Removed formal B0 decay, sustained-B1 taper, and heading-conflict attenuation; restored `22d672e` LF04-direction priority while retaining split diagnostics and the new finish gate.
  - Added the exact failed first-curve regression plus B0 hold, centered route takeover, same-direction cap, and opposing-heading LF04-authority coverage.
  - Re-ran the complete host suite: `18/18 passed`; compiled line control, mission, telemetry, and track application with ARM Compiler 5: `4/4 passed`.
  - Synchronized README, ROBOT_SETUP, HARDWARE_ACCEPTANCE, and TRACK_FUSION_HANDOFF with the corrected LF04-priority behavior and staged first-semicircle acceptance.
### Phase 19: H题第3小题杆球模块融合
- **Status:** complete
- Actions taken:
  - 接管已批准的融合计划；确认只移植源码，不导入压缩包工程、SDK、`empty.syscfg` 或 `ti_msp_dl_config`。
  - 运行会话恢复并复核工作树，保留现有 Phase 18 底盘/赛道未提交修改。
  - 锁定独立杆球工程、PB27/TIMA1_CCP1、UART2 PB15/PB16、TIMG6 1 ms、PB24 启停及默认禁止正式序列的实现边界。
  - 完整核对根目录AGENTS、README、WIRING和ROBOT_SETUP；确认杆球文档必须追加独立章节并保留全部底盘/竞速安全记录。
  - 读取压缩包中的控制器、舵机、协议、标定示例与Maix脚本，列出正式序列安全状态机和TimerA适配缺口。
  - 新增PB27/PINCM58/TIMA1_CCP1板级PWM路由、独立资源所有者和TimerA 50 Hz初始化分支；原TimerG路径未改语义。
  - 移植并加固协议、RDS3230和杆球控制器：正式序列需视觉就绪、一次上电仅一次，丢球/150 ms超时/5 s超限/人工中止均清积分并斜率回中，完成后保持-5 cm闭环。
  - 新增正式/标定应用：PB24启动和中止、开机按住PB24进入需先松键的标定、中心键短按执行1500/1450/1500/1550/1500 us、长按切换0 cm闭环；方向键无舵机动作。
  - 生成独立 `project_ball.uvprojx`，仅含杆球、UART2、TIMG6、TIMA1 PWM、GPIO、OLED和系统依赖；源文件清单无chassis/motor/encoder/LF04/IMU/Fusion。
  - 新增三组主机测试并保留原18组，完整套件为`21/21 passed`；新增/受影响源文件ARMCC 5单文件编译全部通过。
  - 新增`BALL_BALANCE_GUIDE.md`并同步README/WIRING/ROBOT_SETUP/HARDWARE_ACCEPTANCE，明确独立供电、PB27接线、默认序列锁、逐类调参和五次外部验收记录表；未把任何默认方向/端点/PID写成已验证值，也未改AGENTS中的现有stage 3记录。
  - Maix协议脚本测试通过，`maixcam/main.py`与协议脚本`py_compile`通过，杆球工程XML源文件审计通过，`git diff --check`通过。
  - 当前仍有交互式μVision打开`project_track.uvprojx`，按根目录约束未启动并行UV4；三个全量Rebuild继续作为Phase 19C唯一待项。
  - 最终收口复跑保持`21/21 passed`，协议脚本与Maix语法通过，控制器/应用ARMCC 5复编通过，杆球工程禁用源审计及`git diff --check`通过。
  - 用户在μVision完成`project_ball.uvprojx`全量Rebuild：`Code=12652, RO-data=1948, RW-data=880, ZI-data=1304`，`0 Error(s), 0 Warning(s)`；实际build log与map已核对，未出现底盘、电机、编码器、LF04或IMU/Fusion链接符号。
  - 用户关闭交互式μVision后，按杆球→默认→竞速顺序执行命令行构建；杆球目标保持`0/0`，默认工程完整编译为`Code=35244, RO=3068, RW=1072, ZI=29040`且`0/0`，竞速工程编译链接为`Code=41808, RO=3400, RW=1072, ZI=29464`且`0/0`。
