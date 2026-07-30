# Task Plan: H题第2项竞速循迹与UART0遥测

## Goal
在保留40%硬限幅、默认自检和现有安全保护的前提下，实现PA27四路LF04辅助纠偏、竞速速度/用时/PWM遥测及UART0 CSV导出，并完成主机与ARM构建验证。

## Current Phase
Phase 17

## Phases

### Phase 1: Baseline & Interface Audit
- [x] Read AGENTS.md and project documentation
- [x] Capture dirty worktree status
- [x] Confirm exact affected interfaces and tests
- **Status:** complete

### Phase 2: Sensor & Race Control
- [x] Correct right-outer LF04 mapping to PA27/PINCM60
- [x] Add bounded line-assist and lost-line recovery
- [x] Split straight/curve race speeds without changing current baseline
- **Status:** complete

### Phase 3: Telemetry & UART0 CSV
- [x] Expose applied PWM in velocity/chassis status
- [x] Expand telemetry record and session timing
- [x] Preserve braking capture until confirmed stop
- **Status:** complete

### Phase 4: Tests, Docs & Builds
- [x] Update host tests and run full suite
- [x] Update README/WIRING/ROBOT_SETUP/HARDWARE_ACCEPTANCE
- [x] Run diff checks and ARMCC/Keil verification without conflicting with interactive Keil
- **Status:** complete

### Phase 5: Delivery
- [x] Review scope and safety invariants
- [x] Report implementation and remaining hardware acceptance steps
- **Status:** complete

### Phase 6: LF04 Runtime GPIO Protection
- [x] Disable output enable for every GPIO input mode
- [x] Reassert four LF04 pull-up inputs before every sample and surface IO faults
- [x] Preserve chassis stall faults and add alternating raw/normalized OLED diagnostics
- [x] Update tests/docs and rebuild both Keil projects
- **Status:** complete

### Phase 7: LF04 Fixed Polarity & White-Ready Gate
- [x] Fix LF04 normalization to white-high/black-low with constant `WF`
- [x] Require ten consecutive 20 ms `RF` samples plus IMU completion before READY
- [x] Add regression tests for `W7` startup transients and white-gate timing
- [x] Update docs and rebuild both Keil projects
- **Status:** complete

### Phase 8: Track-Geometry Recovery & CSV Repair
- [x] Restrict race-usable LF04 patterns to B1/B2/B4/B6/B8 and latch persistent impossible patterns
- [x] Bridge the fixed LF04 blind zones with bounded last-side recovery while retaining the 200 mm/s loss limit
- [x] Make final approach distance-based and prevent line recovery from raising the mission speed
- [x] Export unwrapped headings, coalesce duplicate timestamps, and append line-state CSV diagnostics
- [x] Add host regressions and update docs
- [ ] Rebuild both Keil projects in the already-open interactive windows
- **Status:** in_progress

### Phase 9: LF04-Only Diagnostic Mode
- [x] Add boot-up Up-key mode selection and stopped-only 60/120/200 speed selection
- [x] Add continuous direct-wheel update API that bypasses fused yaw-rate feedback
- [x] Implement B6 start gate, 400 mm/s² ramp, 1000 mm stop, 500 ms B0 fault, and repeatable stopped workflow
- [x] Add host regressions, update operating documents, and preserve normal race behavior
- [x] Run the full host suite, ARMCC source checks, XML and whitespace validation
- [ ] Rebuild both Keil projects in the already-open interactive windows
- **Status:** in_progress

### Phase 10: LF04 Full-Pattern Tracking
- [ ] Accept B1-B14 centroid measurements with three-cycle side reversal hysteresis
- [ ] Separate B0 loss from B15 ambiguity and apply the selected 60 mm/s / 500 ms policy
- [ ] Replace the exact-B6 diagnostic start gate with three centered centroid samples
- [ ] Update host regressions and operating documents
- [ ] Run the full host suite, ARMCC checks, whitespace validation, and interactive Rebuild handoff
- **Status:** superseded by Phase 11

### Phase 11: Infrared-Priority Steering Correction
- [x] Reverse the currently incorrect grouped left/right wheel bias without changing PA31/PA12/PB8/PA27 mapping
- [x] Make grouped infrared direction authoritative over encoder/IMU steering assistance
- [x] Retune the grouped line controller to P-only with the R500-derived 0.22 ratio and 90 mm/s cap
- [x] Remove the LF-only centered/A-pattern start gate while preserving boot-Up selection and safety gates
- [x] Update host regressions, operating documents, and portable build checks
- [ ] Hand off both full Rebuilds to the already-open interactive Keil windows
- **Status:** in_progress

### Phase 12: LF04-Only Unlimited Lap and 350 mm/s
- [x] Expand stopped-only speed selection to 60/120/200/280/350 mm/s
- [x] Make the default LF04-only distance zero mean unlimited while preserving positive-distance compatibility
- [x] Keep progress/elapsed telemetry and Center-key locked emergency stop unchanged
- [x] Update OLED text, host regressions, and the four operating documents
- [ ] Run the full host suite, ARMCC/XML/whitespace checks, and hand off both Keil Rebuilds
- **Status:** in_progress

### Phase 13: Four-Sensor Centroid and Unlimited B0 Hold
- [x] Replace grouped left/right classification with the physical four-sensor centroid
- [x] Hold the last valid normalized error through B0 without speed reduction or timeout
- [x] Remove obsolete grouped/lost-fault interfaces and preserve infrared-priority route arbitration
- [x] Update full-pattern regressions and the four operating documents
- [x] Raise line `Kp` from 1.0 to 1.5 after the first hardware turning trial
- [x] Run the full host suite plus ARMCC/XML/whitespace and safety-limit checks
- [ ] Hand off both full Rebuilds to the already-open interactive Keil window
- **Status:** in_progress

### Phase 14: Outer-Single Boost and B0 Mode Memory
- [x] Add dedicated 35%/120 mm/s B1/B8 steering configuration
- [x] Remember the last valid LF04 pattern so only B1/B8-to-B0 retains boost
- [x] Preserve infrared-priority route arbitration with dynamic 90/120 mm/s caps
- [x] Make centroid tests follow the configured `Kp=1.8` and cover boost transitions
- [x] Synchronize operating documents and complete portable verification
- [ ] Hand off both full Rebuilds to the existing interactive Keil window
- **Status:** in_progress

### Phase 15: Anti-Wobble PD and LF-Only CSV
- [x] Retune the shared line controller to `Kp=1.2`, `Ki=0`, `Kd=0.20`
- [x] Add one-cycle direct-reversal confirmation without delaying centered or same-side samples
- [x] Preserve immediate B1/B8 boost and defined B0 confirmation/hold behavior
- [x] Add 21-column correction telemetry while retaining 44-byte records and 600 samples
- [x] Finalize and snapshot LF-only telemetry on locked Center stop, then allow stopped UART export
- [x] Update focused/full regressions and the four operating documents
- [x] Complete ARMCC/XML/whitespace portable verification
- [ ] Hand off both full Keil Rebuilds and staged LF-only CSV hardware validation
- **Status:** in_progress

### Phase 16: LF-Only Curve Memory and Smooth B0 Correction
- [x] Add a dedicated LF-only controller configuration without changing race behavior
- [x] Activate symmetric curve memory from confirmed B1/B8 and clear it on B15/opposite confirmation
- [x] Apply 600 ms full boost plus 600 ms taper to the curve-hold bias during B0
- [x] Preserve existing PD behavior when curve memory has not been activated
- [x] Add focused/full regressions and synchronize the four operating documents
- [x] Complete ARMCC/XML/whitespace/safety portable verification
- [ ] Hand off both full Rebuilds to the existing interactive Keil window
- **Status:** in_progress

### Phase 17: LF-Only Curve Exit Travel Guard
- [x] Add a 1200 mm requested-travel exit guard and five-cycle opposite confirmation
- [x] Keep early opposite inner patterns as PD residual without clearing curve direction
- [x] Preserve B0 curve direction and prevent same-sample opposite reactivation
- [x] Replay the failed sequence and cover symmetry, five speeds, B15, and race regressions
- [x] Synchronize the four operating documents
- [x] Complete ARMCC/XML/whitespace/safety portable verification
- [ ] Hand off both full Rebuilds to the existing interactive Keil window
- **Status:** in_progress

## Decisions Made
| Decision | Rationale |
|----------|-----------|
| Keep 360 mm/s as compiled baseline | User selected staged 360→380→400 validation |
| Keep 40%/20000 hard PWM cap and 6000 race feedforward | No speed/PWM evidence yet justifies a duty change |
| Use UART0 D/C CSV workflow only | Explicit user requirement |
| LF04 remains auxiliary to encoder+IMU | Prevents transverse A line and sensor loss from replacing lap gates |
| Reassert LF04 input mode at every sample | User observed all four module outputs low only while race PWM was active; software must actively clear any stale DOE state without requiring Watch |
| Fix LF04 polarity to white-high/black-low | Hardware confirms white raw `RF`; adaptive startup capture incorrectly stored transient `W7` |
| Treat only B1/B2/B4/B6/B8 as race-usable | The official 16-20 mm line and fixed 33/14.5/33 mm LF04 spacing make other nonzero patterns physically implausible during normal tracking |
| Preserve B0 slowdown but search toward the last reliable side | The fixed module has 13-17 mm blind zones between each inner and outer sensor |
| Keep the existing 15 mm stop lead until retest | The uploaded lap reached the gate near 200 mm/s, so its 41 mm overrun is not valid 100 mm/s calibration data |
| Enter LF04-only mode by holding Up at boot | Prevents control-source changes while moving and keeps Center as the only start/emergency-stop key |
| Select 60/120/200 only while stopped | Makes staged testing repeatable without reflashing and prevents runtime speed jumps |
| Use direct wheel-speed mode for LF04-only steering | Bypasses fused yaw-rate feedback while retaining encoder PID, odometry, stall protection, and hard limits |
| Latch sustained B0 at 500 ms | Covers the estimated 283 ms worst blind-zone crossing at 60 mm/s but prevents the observed multi-second one-sided search |
| Supersede Phase 10 centroid/B15 policy | User explicitly requires four inputs grouped as two: B1-B15 are valid grouped states and only B0 is lost |
| Make infrared direction authoritative | Encoder/IMU assistance may reinforce but must never cancel or reverse the grouped infrared wheel bias |
| Use P-only grouped steering at 0.22 ratio, 90 mm/s cap | Binary left/center/right error does not justify integral/derivative history; the initial ratio matches the PDF R500 geometry |
| Use five LF-only speeds through 350 mm/s | User selected 60/120/200/280/350 for staged hardware validation |
| Use `distance_mm=0` as unlimited | Removes the default 1000 mm stop without breaking the existing positive-distance state-machine interface |
| Keep Center as a locked emergency stop | User explicitly chose manual locked stopping at the end of the lap rather than automatic or repeatable graceful stopping |
| Use physical LF04 centers `-40.25/-7.25/+7.25/+40.25 mm` | The module drawing fixes the 33/14.5/33 mm center spacing and the user requested true four-sensor control |
| Apply four-sensor control to both modes | User selected one shared controller for LF-only diagnostics and formal racing |
| Hold the last normalized error at full task speed through B0 | User explicitly selected no 120 mm/s reduction and no 300 ms fault; a startup B0 defaults to centered error |
| Increase the four-sensor P gain while retaining limits | After the car failed to turn tightly enough, raise to `Kp=1.5`, keep `Ki=0`, `Kd=0`, PID output `±1`, 22% ratio, and 90 mm/s cap |

## Errors Encountered
| Error | Attempt | Resolution |
|-------|---------|------------|
| None | 1 | N/A |
| Host suite stopped compiling `line_sensor_test` after PA27 test change | 1 | Diagnose direct GCC output, then patch the test stub |
| Stop gate used average wheel speed rather than both wheel thresholds | 1 | Require left and right independently below 20 mm/s and add a reset regression |
| Host GPIO test pulled in the MCU SDK header, then lacked `DIN31_0` in its isolated register stub | 1 | Force-include the minimal header for that test and complete the GPIO register stub |
| First Phase 8 host compile placed update-state locals in the wheel-limit helper | 1 | Read the numbered source area and move the declarations into `chassis_track_line_assist_update()` |
| First Phase 8 line-assist run expected the old one-cycle derivative values after adding slew limiting | 1 | Update assertions to the 18.125 mm/s steady inner correction and 58.125/60 mm/s blind-zone ramp |
| Phase 8 combined documentation patch did not match one existing telemetry bullet verbatim | 1 | Inspect the exact section and apply smaller file-specific patches against the current text |
| Windows `rg` rejected literal `user/Objects/*.htm` path arguments during final audit | 1 | Search the directory path and use `-g '*.htm' -g '*.txt'` filters instead |
| Windows `rg` rejected literal source wildcard paths in the safety audit | 1 | Search the containing directories with exact filename globs passed through `-g` |
| Phase 8 plan update temporarily duplicated four completed checklist items | 1 | Remove the stale unchecked copies and retain the checked items plus pending interactive Rebuild |
| Final line-number lookup used an invalid literal Windows test wildcard | 1 | Search the `tests` directory with `-g 'chassis_*test.c'` |
| Phase 9 bookkeeping patches used incorrect cross-file context twice | 1-2 | Switch to one exact patch per planning file; source and tests were unaffected |
| Phase 9 source-inspection batch failed because `rg` received a literal Windows `*.ps1` path | 1 | Re-run file reads separately and search directories with `-g '*.ps1'` |
| First Phase 9 running-loop patch did not match the current nested command block | 1 | Split the integration into small exact patches using the current numbered source |
| Phase 11 baseline host suite did not link `chassis_track_line_control_test` because `code/pid.c` was omitted | 1 | Add the dependency to that host-test case as part of this phase |
| Combined hardware-acceptance patch did not match the exact existing LF04 wording | 1-2 | Re-read the focused section, then split every paragraph into an independent exact patch |
| Phase 14 focused GCC output directory did not exist | 1 | Write the focused executable to the system temporary directory like the host runner |
| Phase 15 combined bookkeeping patch used progress context in task_plan | 1 | Split the task and progress updates into exact file-specific patches; source changes were unaffected |
| Phase 15 combined four-document patch missed one ROBOT_SETUP UART line | 1 | Apply one exact file-specific patch at a time using the current text; no documentation hunk was applied |
| Phase 15 README/test cleanup patch used the plural test-function name | 1 | Re-read the exact function and apply a surgical patch against `test_invalid_input` |
| Phase 15 combined final check treated `rg` zero matches as failure | 1 | Separate the host suite and make the expected no-match audit handle exit code 1 explicitly |
| Phase 16 safety audit tried to splat a hashtable property as `@c.Paths` | 1 | Assign the property to `$auditPaths` first, then splat that variable; the corrected audit passed |
| Phase 17 combined task/progress bookkeeping patch matched the repeated phase heading ambiguously | 1 | Split the update into exact file-specific patches; source and tests were already successful |
