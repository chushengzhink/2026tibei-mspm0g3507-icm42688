# Progress Log

## 2026-07-31 Phase 30
- **Status:** implementation_complete_waiting_for_hardware
- 已按三份1300 us新CSV将+5允许误差改为±1 cm、确认时间改为30 ms，并只在最终-5阶段应用≤1 cm/s低速门。
- 已更新无效帧清零、高速+5折返和最终-5低速稳定的测试契约，以及AGENTS、README、ROBOT_SETUP和BALL_BALANCE_GUIDE当前说明。
- PID、1300–1700 us、脱困、5秒门、Maix、26列CSV和RAM布局未改；按用户要求未运行任何回归、编译或差异检查。


## 2026-07-31 Phase 29
- **Status:** in_progress
- 解析用户提供的三份26列CSV：三次均在`TO_PLUS_5`超时，峰值+8.372/+12.392/+11.961 cm，正式脱困实际达到1000 us且随后普通控制长期饱和制动。
- 用户确认恢复1300–1700 us，不再保留正式+5的1000 us例外；本轮仅修改绝对下限和正式脱困下限，不调整位置环、速度环、观察器或序列判定。
- 用户要求未明确提出检查时不主动执行测试、编译或差异检查；本轮修改后直接交付，由用户自行Rebuild/Download并取得新CSV。
- 首个跨文件组合补丁因progress上下文不精确而整体未应用；已改用稳定锚点拆分补丁。
- 已将`BALL_SERVO_MINIMUM_US`和`BALL_SEQUENCE_BREAKAWAY_SERVO_MINIMUM_US`统一改为1300 us，并增加编译期范围约束，防止正式脱困再次越过1300–1700 us。
- 已同步控制回归常量、AGENTS/README/ROBOT_SETUP/BALL_BALANCE_GUIDE及文件规划记录；PID、序列判定、Maix、遥测协议和RAM布局均未修改。
- 按用户要求未运行主机测试、ARMCC、Keil构建或差异检查；下一步由用户在现有project_ball窗口Rebuild/Download并采集三份新CSV。


## 2026-07-31 Phase 26
- 恢复中断前工作区，确认`BALL_SEQUENCE_OVERSPEED_BRAKE_ENABLED=0`、默认关闭回归和26列文档修改均保留，未覆盖任何底盘或Maix在途改动。
- 解析固定新球的三份CSV：一次高速穿区被误判完成，另两次因低阻尼在+5侧振荡超时；视觉、控制周期和独立制动关闭状态正常。
- 锁定最小实现边界：位置D=0.40，加统一1.0 cm/s端点低速门；不改P/I、速度环、脱困、5秒门、舵机、遥测或协议。

## 2026-07-31 Phase 21
- 接管位置P降档与杆球UART0 CSV计划；复核dirty worktree并保留全部用户底盘/竞速改动。
- 已锁定实现边界：独立杆球遥测、约6秒RAM记录、失视跨段保留、PB24中止后记录回中、1500 us后才允许D/C。
- 新增独立`ball_telemetry`模块：36字节定点记录×600、19列CSV、D/C门控、1秒BUSY限频；未引用底盘遥测。
- 位置P已单独从1.6降至0.8，其他PID/限幅未改；UART0 ISR已对杆球构建启用。
- 首次24组回归在旧控制周期断言停止：旧测试要求位置目标为正时控制量必为正，忽略了内环对残余正速度的制动；已改为验证控制量与速度误差同号，并显式锁定0.8/0/0与25/0。
- 第二次回归在编译该固定值检查时发现测试文件未直接包含配置头；已补充`ball_balance_config.h`，生产源码不受影响。
- 位置控制、遥测、自动中心和速度回退四个聚焦测试均通过；ARMCC 5.06单编新增模块、三种应用条件分支和杆球ISR均无诊断。
- 已同步README、BALL_BALANCE_GUIDE、WIRING、ROBOT_SETUP和AGENTS，记录P=1.6发散、P=0.8回退及UART0 D/C接线与安全门。
- 完整主机回归`24/24 passed`；Maix协议/Python语法、project_ball XML/禁用源、`git diff --check`全部通过。
- 36×600记录占21600字节；按旧RW+ZI估算总23808/32768字节，保留约8960字节链接余量。用户明确自行在现有project_ball窗口Rebuild，因此未操作或启动UV4。
- 用户完成project_ball全量Rebuild：`Code=13988, RO=2188, RW=896, ZI=22928`，`0 Error(s), 0 Warning(s)`；实际map中UART0 ISR和ball_telemetry符号齐全，禁用底盘/电机/编码器符号审计通过，实际RW+ZI为23824字节。

## 2026-07-31 Phase 20
- 接续已完成的自动0 cm中心闭环源码、测试与文档修改，未改动PID、Maix映射、底盘或电机代码。
- `git status --short`确认工作区仍包含大量用户未提交改动，全部保留。
- 全局搜索未发现旧宏`BALL_AUTO_SPEED_TEST_ON_BOOT`；新三态宏和OLED状态文本均已落到源码、测试与文档。
- 检测到现有`project_track`和`project_ball`两个μVision窗口，后续不启动并行UV4。
- 完整主机回归通过`23/23`；Maix协议脚本与Python语法检查通过；`git diff --check`仅报告已有LF/CRLF提示，无空白错误。
- ARMCC命令盘点第一次因Windows不接受字面量`*.ps1`路径而退出；已记录并改用目录加`-g`过滤，不重复该命令。

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

### Phase 22: 正式+5到-5首轮测试入口
- **Status:** in_progress
- Actions taken:
  - 读取根目录约束、代码修改规范和文件规划说明，恢复现有任务上下文并保留全部未提交底盘/杆球改动。
  - 核对正式入口、遥测条件编译、OLED刷新和PB24分支，确认不能只修改自动模式宏。
  - 锁定最小修改边界：PID、Maix协议/映射、舵机限幅、序列判定和静摩擦补偿控制层均保持不变。
  - 一次批量接口读取因附带搜索不存在的`tests/stubs/ml_common.h`而返回退出码1；需要的`ball_demo`接口已读到，后续改用真实公共头路径。
  - 首次组合应用补丁因OLED条件编译上下文不匹配而整体未应用；改用精确的小补丁，不重试同一方式。
  - 已把默认模式切换为正式人工入口；正式模式现初始化UART0遥测、成功启动时开始会话、运行/完成保持期间冻结OLED周期刷新，并允许完成态PB24关闭闭环后斜率回中。
  - 遥测结束与导出门已统一为“控制关闭、舵机目标及实际均回到1500 us”；自动速度/中心模式的原有锁存语义保持不变。
  - 应用测试现支持模式0/1/2三种编译；新增正式等待/启动、活动期OLED冻结、完成态单行提示、PB24回中、运行中止、失视结束、导出门和开机标定隔离覆盖。
  - 控制器回归增加评分序列持续运行时静摩擦补偿始终关闭的断言；PID与控制源码未改。
  - 完整主机回归通过`25/25`，其中新增的正式模式应用测试与原自动中心/速度模式测试均通过。
  - 已同步README、ROBOT_SETUP、BALL_BALANCE_GUIDE和AGENTS的当前模式、PB24完成态、OLED冻结、正式遥测及首轮未验收声明；历史中心调参记录保留。
  - ARMCC 5.06使用`project_ball`实际宏和包含路径单编`ball_balance_app.c`无诊断；工程XML源清单继续不含底盘、电机、编码器、LF04或IMU模块。
  - 控制器完成态测试补充验证`ball_balance_enable(false)`会回中但保留`COMPLETE`和锁定用时，与正式PB24完成态操作一致。
  - 最终完整主机回归再次通过`25/25`；Maix协议、Python语法、project_ball XML、`git diff --check`和ARMCC 5.06单编均通过。
  - 检测到用户交互式UV4窗口仍打开，因此未启动并行命令行Rebuild；本阶段只剩用户在现有窗口全量Rebuild/Download和实机首轮测试。
### Phase 23: 正式脱困斜坡加速与反向像素抗抖
- **Status:** in_progress
- Actions taken:
  - 恢复文件规划上下文并确认脏工作树，保留全部用户在途底盘与杆球改动。
  - 锁定最小实现边界：正式斜坡100 us/s、中心斜坡50 us/s、反向位移与速度双门限；不修改PID、1300 us下限、Maix或CSV。
  - 增加正式序列专用100 us/s配置和阶段感知斜坡选择，中心调试继续使用原50 us/s。
  - 将激活态反向清除改为锁存方向下“背离至少0.15 cm且反向速度超过0.3 cm/s”同时成立；单独速度尖峰继续冻结积分和保留增力。
  - 增加正式首周期1.0 us、中心0.5 us、反向小位移抗抖和真实背离清除回归；首次完整主机套件通过`25/25 passed`。
  - 同步README、杆球指南、ROBOT_SETUP和AGENTS：记录两份1300 us固件最低1329/1327 us及TIMEOUT、正式100/中心50 us/s和反向双门限；保留历史实验参数可追溯性。
  - 使用project_ball实际宏和包含路径，以ARM Compiler 5.06分别单编正式补偿启用配置和`BALL_SEQUENCE_BREAKAWAY_ENABLED=0`回退配置，均无诊断。
  - project_ball XML隔离检查通过：38个源文件中不存在底盘、电机、编码器、LF04或IMU模块；`git diff --check`通过。
  - Phase 23便携实现与验证完成；保留用户现有μVision窗口，全量Rebuild/Download和三次实机CSV作为交接项。

### Phase 24: 正式最大脱困与超速强制制动
- **Status:** in_progress
- Actions taken:
  - 恢复现有文件规划和脏工作树，保留全部用户底盘、杆球及Maix在途改动。
  - 解析两份100 us/s CSV并锁定直接150 us正负脱困、0.5/0.1 cm/s超速制动迟滞和26列CSV方案。
  - 核对控制器重置路径及遥测布局，确认可在不增加44字节记录和600条RAM容量的前提下增加制动标志。
  - 增加正式直接最大增力开关和超速制动开关；正式±5触发后直接150 us，中心仍按原斜坡运行。
  - 增加阶段锁定0.5/0.1 cm/s迟滞制动，+5请求1650 us、-5请求1300 us，制动当周期冻结积分并由统一重置路径清除。
  - 状态接口增加`brake_active`；遥测用原单字节flags封装脱困/制动位，CSV扩展为26列且44字节记录尺寸不变。
  - 首次完整主机回归在第21组按预期停于旧的正式首周期1 us断言；前20组通过，下一步只更新正式直接加力契约并增加制动测试。
  - 更新正式增力断言并新增0.49/0.51进入门、0.3迟滞保持、0.1退出门、正负制动边界、积分冻结及安全清零测试；完整主机回归恢复`25/25 passed`。
### Phase 25: 撤销全程极限制动并恢复连续速度环制动
- **Status:** in_progress
- Actions taken:
  - 读取根目录约束、代码修改规范和文件规划说明，执行会话恢复、`git status --short`与差异统计，确认保留全部用户在途修改。
  - 完整解析三份新26列CSV并结合机械结构照片复核：直接脱困有效，独立1650 us制动在离+5仍约4 cm时过早触发并制造极限往返。
  - 锁定最小修改边界：默认关闭独立制动，保留直接150 us脱困、普通PID、26列CSV、RAM布局、Maix和全部安全门。
  - 将`BALL_SEQUENCE_OVERSPEED_BRAKE_ENABLED`默认值改为0，保留原实现供宏覆盖诊断。
  - 扩展同一控制回归：默认关闭时正负阶段超速均保持`brake_active=0`，由速度环产生对称、非边界覆盖的连续制动，位置积分不再被独立制动冻结。
# Phase 27: 串级PID收口与正式1000 us脱困（已由Phase 29撤销）
- **Status:** in_progress
- 已读取根目录约束、项目文档、文件规划与手术式编码指导，保留全部用户未提交改动。
- 只读审计确认速度环27/3的控制符号正确，正式入口已是串级；发现750 us被普通±200 us限幅挡住且50000 ms违背正式5秒门。
- 修改前完整主机回归为24/25，唯一已报告失败为杆球参数断言仍期待旧位置环和速度环数值。
- 首次实现后完整回归仍为24/25：旧端点测试只固定等待700 ms，观察器速度尚未满足新增1 cm/s门；生产判定不改，测试改为在5秒预算内等待真实低速完成。
- 第二次完整回归仍为24/25：速度环27/3在合成超速样本上正确达到-200 us限幅，旧断言仍要求大于-150 us；改为验证方向正确且不越过普通±200 us边界。
- 一次组合补丁因多余空hunk格式被apply_patch拒绝，未应用任何修改；已改用有效的精确补丁。
- 聚焦测试继续暴露同一旧量级假设：速度27/3可让普通制动目标达到1700 us，旧断言要求低于1650 us；生产仍受普通1300～1700 us约束，测试改为验证全局上限。
- 对称负向聚焦样本同样正确达到+200 us普通限幅；旧严格小于断言改为允许等于限幅。

# Phase 28: 第三问空CSV遥测修复
- **Status:** in_progress
- 已读取根目录约束、项目交接、文件规划和手术式编码指导，保留全部现有未提交修改。
- 已确认现象为D导出只有26列表头；当前导出器对0条记录没有显式EMPTY诊断。
- 已用当前AXF符号排除26400字节记录区、计数器地址和唯一32 KB SRAM布局冲突。
- 锁定最小边界：只改遥测启动/导出/OLED/测试/文档，不改串级PID、Maix、5秒门或舵机安全参数。
- 首次跨文件组合补丁因hunk分隔格式错误被拒绝，未写入源码；后续改用逐文件精确补丁。
- 已将正式遥测启动从PB24成功分支移到序列实际进入TO_PLUS_5/TO_MINUS_5后的统一状态门，并保留同轮询首条记录。
- 已增加终态OLED的`CSV Nxxx D=OUT`/`CSV EMPTY REBOOT`分流；空缓存D现在返回`EMPTY`而非空表头。
- 遥测单元测试已加入大小写D空缓存、重复EMPTY及26列逐行校验；应用遥测桩开始维护真实样式的记录数量。
- 500 ms脱困故障回归已把边界搜索改为10 ms控制粒度，生产故障逻辑未改。
- 新增生产`ball_balance_app.c + ball_telemetry.c`联合主机回归，覆盖PB24启动、+5/-5阶段、终态回中、非空26列导出、重复D、C清空及清空后EMPTY。
- 首次26组完整回归停在`ball_balance_test.c:1554`；原1513行500 ms计时已通过，新的失败是负方向旧测试仍把到边界后的增力量硬编码为至少95 us。
- 已将负方向旧断言收口为“增力为正且不超过150 us”，保留300 ms未故障和停用清零契约；生产控制未改。
- 第二次完整主机回归`26/26 passed`，新增真实应用+遥测联合测试及全部原底盘/杆球测试均通过。
- 已同步README、ROBOT_SETUP、BALL_BALANCE_GUIDE和AGENTS：正常终态显示非零记录数，EMPTY必须重新上电重跑，禁止用空表头调参。
- ARMCC 5.06按project_ball真实宏/包含路径单编应用与遥测均零诊断；用户随后在现有μVision窗口完成全量Rebuild，未启动第二个UV4。
# Phase 31: 第三问负端捕获与停滞恢复重构
- **Status:** complete
- 已读取用户给出的项目约束，执行`git status --short`并确认仅有三个未跟踪tmp产物，未回滚或清理任何现有文件。
- 已读取文件规划与手术式编码指导，恢复旧任务记录并建立本轮独立阶段。
- 已锁定用户要求：本轮不运行回归、编译、差异检查或Keil构建。
- 已完整读取根目录`AGENTS.md`并确认当前负端捕获/阻尼基线比对话粘贴尾部更新。
- 已完整读取`README.md`、`WIRING.md`和`ROBOT_SETUP.md`，锁定当前书面控制契约与安全约束。
- 已解析三份独立CSV的时序、阶段切换、位置/速度范围及终态；确认共同失败点是负端满增力脱困后提前进入状态5，而不是捕获后保持不足。
- 已定位杆球正式状态机、配置、应用和遥测源文件；下一步逐行审计负向制动与脱困故障路径。
- 已读取控制配置和公开状态枚举，确认CSV状态5为ABORTED，负端当前复用了全局脱困故障机制。
- 已逐行审计控制器前半部、负向速度参考、捕获判定和脱困故障路径，确认轨迹制动锁存与脱困互相独立是结构性缺口。
- 已逐100 ms重放三轮负向轨迹并锁定闭合修正方案：消除±0.4至±1.0 cm死区、负端1650/1700两级恢复、完成门绑定真实捕获、+5确认缩至10 ms。
- 已修改控制配置和状态机：负端2.0 cm固定接近区、未捕获恢复阈值±0.4 cm、1650 us/150 ms后升级1700 us/350 ms、完成门绑定捕获态；+5确认缩为10 ms。
- 全部改动保持舵机1300–1700 us绝对范围、500 ms最大脱困总保持、5000 ms总超时、视觉和PB24安全路径不变。
- 已同步README、ROBOT_SETUP、BALL_BALANCE_GUIDE和AGENTS中的当前基线与三份CSV结论；历史实验记录保留。
- 按用户要求未运行回归、编译、`git diff --check`或Keil构建，本阶段以待Rebuild/Download实测状态交接。

# Phase 32: 第三问双端轨迹能量收敛
- **Status:** complete
- 已确认工作树只包含上一轮杆球、文档和规划在途修改，未清理或覆盖任何用户文件。
- 已解析三份新CSV：正端问题是高能量入带，负端问题是提前制动后回弹；现有两级恢复不再提前ABORT。
- 已锁定实现参数：正向3.0 cm/s、2.5 cm接近区、1.2倍参考；负向3倍接近、±1 cm低速捕获；其余安全边界不变。
- 已修改生产控制参数和正向速度参考：正端在停车距离门或2.5 cm固定区任一命中时进入1.2倍连续接近；负端使用3倍接近并在题面±1 cm低速捕获。
- 现有1650/1700 us两级恢复、500 ms脱困总门、5000 ms序列门和全部安全接口未改。
- 已审计主机测试中的序列参数和负端捕获场景，确认需同步旧硬编码断言及30 ms确认节奏。
- 已同步`ball_balance_test.c`的当前参数契约、2.5 cm正向接近区、10 ms折返恢复和题面±1 cm低速捕获场景。
- 已同步README、ROBOT_SETUP、BALL_BALANCE_GUIDE和AGENTS；WIRING因接线与供电无变化而保持不动。
- 按用户要求未运行主机回归、编译、`git diff --check`、Keil构建或其他验证命令；等待现有project_ball窗口Rebuild/Download和三份新CSV。
