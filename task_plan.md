# Task Plan: H题第2项竞速循迹与UART0遥测

## Goal
在保留40%硬限幅、默认自检和现有安全保护的前提下，实现PA27四路LF04辅助纠偏、竞速速度/用时/PWM遥测及UART0 CSV导出，并完成主机与ARM构建验证。

## Current Phase
Phase 34

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

### Phase 18: Formal IMU + LF04 + Encoder Arbitration
- [x] Add CSV-derived replay regressions for signed route/heading/line arbitration
- [x] Split mission feedforward and heading feedback outputs and add a 360 +/- 5 degree finish window
- [x] Restore LF04-direction authority after the feedforward-first hardware failure
- [x] Expand telemetry to a 52-byte public record and 27 CSV columns while retaining 600 compact RAM records
- [ ] Complete both interactive Keil Rebuilds and staged hardware verification
- **Status:** in_progress

### Phase 19: H题第3小题杆球模块融合
- [x] 19A: 从压缩包仅提取并适配杆球控制、RDS3230、Maix协议与脚本，增加PB27/TIMA1_CCP1 PWM路由
- [x] 19B: 新增杆球正式/标定应用入口与独立 `project_ball.uvprojx`，确保不编译或初始化底盘电机
- [x] 19C: 增加协议、舵机和控制状态机主机测试，完成脚本、ARMCC、XML、空白及三工程构建验证
- [x] 19D: 完成接线与分阶段实机标定文档交接，不把未验证参数记录为实测值
- **Status:** complete

### Phase 20: 自动0 cm位置P外环首轮测试
- [x] 将上电自动入口改为关闭/速度内环/0 cm串级闭环三态，默认0 cm串级闭环
- [x] 保持位置P/D/I=1.6/0/0与速度P/D=25/0，仅切换控制模式
- [x] 增加正负位置方向、限幅、失视恢复、PB24锁存中止和不启动正式序列的回归测试
- [x] 同步杆球操作与实测记录文档
- [ ] 完成23组主机测试、Maix协议/语法、空白和ARMCC验证
- [ ] 在现有project_ball μVision窗口全量Rebuild并核对无底盘电机链接符号
- **Status:** in_progress

### Phase 21: 位置P降档与UART0 CSV遥测
- [x] 将位置P从1.6单独降至0.8，保留全部其他PID和安全限幅
- [x] 新增杆球独立600条/10 ms紧凑遥测及UART0停机导出门
- [x] 集成自动中心入口、失视记录、PB24中止后回中完成与D/C命令
- [x] 增加主机测试并同步project_ball、ISR和杆球文档
- [x] 完成主机、Maix、ARMCC、空白、全量Rebuild及链接符号验证
- **Status:** complete

### Phase 22: 正式+5到-5首轮测试入口
- [x] 将默认入口从自动0 cm闭环切换为PB24一次启动的正式序列
- [x] 在正式模式保留600条UART0遥测、完成后PB24回中与安全导出门
- [x] 冻结正式活动闭环OLED周期写入，并仅在完成后显示一次状态
- [x] 增加正式应用回归，保留自动速度/中心模式测试
- [x] 运行全部主机、Maix、ARMCC、XML、空白与链接隔离检查
- [x] 同步杆球操作文档并交接用户全量Rebuild/Download
- [ ] 用户在现有μVision窗口全量Rebuild并Download正式序列固件
- **Status:** in_progress

### Phase 23: 正式脱困斜坡加速与反向像素抗抖
- [x] 正式序列脱困斜坡改为100 us/s，中心调试保持50 us/s
- [x] 反向清除增加0.15 cm背离位移门，保留速度门与正向释放条件
- [x] 增加正式/中心斜坡及反向像素抖动回归测试
- [x] 同步杆球文档并完成主机、ARMCC、工程隔离与空白验证
- **Status:** complete

### Phase 24: 正式最大脱困与超速强制制动
- [ ] 正式±5阶段静止300 ms后直接施加±150 us，中心调试保持50 us/s
- [ ] 增加0.5/0.1 cm/s迟滞的阶段锁定反向制动并覆盖全部安全清零路径
- [ ] CSV增加`brake_active`且维持600条×44字节单SRAM布局
- [ ] 更新控制/遥测/应用回归与杆球文档
- [ ] 完成25组主机测试、ARMCC启用/回退、工程隔离及空白检查
- **Status:** in_progress

### Phase 25: 撤销全程极限制动并恢复连续速度环制动
- [ ] 默认关闭正式序列独立超速极限制动，保留直接150 us脱困与普通串级PID
- [ ] 保持26列CSV、44字节记录和`brake_active`兼容接口不变
- [ ] 更新默认关闭/诊断回退测试及杆球文档
- [ ] 完成25组主机测试、ARMCC默认/回退单编、工程隔离及空白检查
- **Status:** in_progress

### Phase 26: 新球位置D增阻尼与端点低速确认
- [ ] 将位置D从0.20单独提高到0.40，保持P/I、速度环和脱困策略不变
- [ ] 为+5/−5持续计时增加`|速度|<=1.0 cm/s`门并在任一条件越界时清零
- [ ] 更新端点穿越、正负速度、D幅值和原安全路径回归
- [ ] 同步新球实测与回退值文档并完成便携验证
- **Status:** in_progress

### Phase 27: 串级PID收口与正式1000 us脱困（已由Phase 29撤销）
- [ ] 恢复位置P/D/I=0.8/0.40/0.30和5000 ms总超时，锁定速度P/D=27/3
- [x] 历史上曾仅对正式+5开放1000 us；该例外已被Phase 29撤销，当前全路径恢复1300～1700 us
- [ ] 锁定正式启动、折返、重捕获和完成保持的串级模式及低速端点门
- [ ] 更新PID、速度D、脱困边界与安全清零回归并同步杆球文档
- [ ] 完成25组主机测试、ARMCC配置单编、工程隔离和空白检查
- **Status:** in_progress

### Phase 28: 第三问空CSV遥测修复
- [ ] 将正式序列遥测启动集中到实际TO_PLUS_5/TO_MINUS_5状态并立即记录首条
- [ ] 空缓存D命令返回EMPTY，终态OLED显示记录数量或CSV EMPTY REBOOT
- [ ] 保持26列、44字节、600条布局和全部串级PID/安全参数不变
- [ ] 增加生命周期、空缓存、重复导出、满缓存和26列回归
- [ ] 修正500 ms脱困故障测试计时并完成主机、ARMCC与空白检查
- [ ] 同步第三问导出操作文档并等待交互式Keil全量Rebuild/Download
- **Status:** in_progress

### Phase 29: 第三问恢复1300–1700 us安全边界
- [x] 根据三份26列CSV确认1000 us正式脱困导致过强冲击，三次均停留在TO_PLUS_5并超时
- [x] 将舵机绝对下限与正式+5脱困下限统一恢复为1300 us，保持1700 us上限
- [x] 保持位置P/D/I=0.8/0.40/0.30、速度P/D=27/3、5秒门和端点低速判定不变
- [x] 更新控制回归常量与当前项目/杆球操作文档，撤销1000 us当前例外
- [ ] 等待用户Rebuild/Download并采集三份新26列CSV
- **Status:** in_progress

### Phase 30: 第三问+5折返门修正
- [x] 解析三份1300 us新CSV并确认第1/3次已进入+5±1 cm但被低速200 ms门拒绝折返
- [x] 保持1300–1700 us和全部PID，将+5改为±1 cm位置连续确认30 ms且不检查速度
- [x] 保持最终-5的±1 cm、≤1 cm/s连续500 ms稳定门和5000 ms总超时
- [x] 更新测试契约与当前项目/操作文档，不改变API、CSV、RAM或Maix
- [ ] 等待用户Rebuild/Download并采集新CSV
- **Status:** in_progress

### Phase 31: 第三问负端捕获与停滞恢复重构
- [x] 解析三份最新26列CSV，逐段确认+5折返、-5进入带、速度收敛和约-4 cm停滞的共同原因
- [x] 审计当前正式轨迹、端点稳定门、积分/脱困复位及最终保持逻辑，找出状态机缺口
- [x] 用完整的负端“接近-捕获-保持-停滞恢复”逻辑替代零散门限补丁，同时保持1300–1700 us、5秒门和PB24安全路径
- [x] 同步必要的项目说明与本轮实测结论
- [x] 按用户要求不运行回归、编译、差异检查或Keil构建
- **Status:** complete

### Phase 32: 第三问双端轨迹能量收敛
- [x] 将+5轨迹改为3.0 cm/s巡航、2.5 cm固定接近区和1.2倍连续位置参考
- [x] 将-5接近增益提高到3倍，并按题面±1 cm低速捕获后累计500 ms完成
- [x] 保持现有1650/1700 us两级恢复、5000 ms总门及全部安全路径
- [x] 同步主机测试契约和双端轨迹场景，但按用户要求不运行任何验证
- [x] 同步当前项目说明、操作指南和实测记录
- **Status:** complete

### Phase 33: 第三问1525 us机械水平零点
- [x] 分离1500 us安全回中与1525 us闭环水平零点，并覆盖正常READY待机
- [x] 将闭环控制、制动和脱困的控制量/脉宽换算统一到1525 us基准，保留绝对脱困脉宽
- [x] 朝目标速度达到释放速度门时清零脱困边界故障计时，避免移动中误ABORT
- [x] 同步控制与应用测试契约并完成26组主机回归
- [x] 同步当前项目说明、操作指南和实测记录
- **Status:** complete

### Phase 34: H题第4/5/6问底盘与滚球融合
- [x] 收口1525 us测试、文档与规划契约，恢复现有26组主机测试
- [x] 新增H4/H5/H6任务状态机、单固件模式/目标选择和联合应用入口
- [x] 新增40字节×600条联合遥测并确保唯一32 KB SRAM内不出现双缓冲
- [x] 新增联合任务、应用和遥测主机测试，保持现有三工程回归不变
- [x] 新增`project_h456.uvprojx`并完成ARMCC、XML、差异与安全边界检查
- [x] 同步README、WIRING、ROBOT_SETUP和AGENTS实车操作/验收说明
- [ ] 保留现有μVision窗口，由用户在交互式窗口全量Rebuild联合工程
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
| Keep split route/heading diagnostics but restore LF04-direction authority | The feedforward-first trial reversed a live B8 `-120 mm/s` recovery request into `+65..+84 mm/s` and left the track before the first semicircle ended |
| Keep formal B0 at full remembered LF04 authority | The 120 ms decay removed the only lateral reacquisition signal; formal `line_weight_pct` is now fixed at 100 |
| Use a 360 +/- 5 degree, three-cycle finish window | Rejects the observed 376 degree finish while preserving the route+50 mm safety stop |
| Start stop-lead calibration at 36 mm with a 190 mm approach | The authoritative lap coasted 36.2 mm after the stop command; the longer approach preserves the 400 mm/s2 deceleration distance |
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
| Phase 18 combined planning-file patch missed a Unicode progress line | 1 | Split the bookkeeping into independent stable-anchor patches; no source file was touched |
| Phase 18 first integrated host run reported 14/18 | 1 | Core sources compiled; update the intentionally changed telemetry and finish-gate contracts, then add formal-fusion replay coverage |
| Phase 22 interface batch searched a nonexistent `tests/stubs/ml_common.h` | 1 | Keep the successfully read `ball_demo` interface and use the real `ml_libs/ml_common.h` path for subsequent checks |
| Phase 22 combined app/config patch missed the exact display conditional context | 1 | No hunks were applied; split the change into small source-specific patches against the numbered current file |
| Initial Phase 19 inventory command returned exit code 1 when no `UV4` process matched | 1 | Treat the empty process result separately from file inventory; no build was launched |
| First focused ball controller test expected integral growth while the +5 cm command was saturated | 1 | Move the simulated ball to +2 cm before checking nonzero integral, then keep the clear-on-abort assertion |
| First focused-test shell command was rejected because it combined cleanup with a computed temporary path | 1 | Use explicit executables under the system temporary directory and leave cleanup outside the test command |
| Ball-stub inventory requested a nonexistent shared `ml_pwm.h` | 1 | Add isolated `tests/ball_stubs` headers so existing chassis test stubs remain unchanged |
| Cleanup of generated ARMCC/Python directories was rejected by the command safety policy | 1-2 | Add narrow ignore rules for `tmp/ball_armcc/` and `__pycache__/`; no user files are removed |
| Windows `rg` rejected a literal `code/ball_*.c` path during the forbidden-call audit | 1 | Search the `code` directory with `-g 'ball_*.c'`; the corrected audit found no forbidden calls |
| First 52-byte-by-600 telemetry allocation overflowed SRAM by `0xC10` | 1 | Keep the 52-byte public record and 27-column CSV but store 600 internal 44-byte compact records; tick/heading remain exact and x/y use 0.25 mm resolution |
| Feedforward-first formal arbitration caused severe first-semicircle departure | 1 | Restore `22d672e` LF04-priority steering, retain split telemetry/finish changes, and add the exact B8 `-120` versus `+77/+37` regression |
| Phase 20 ARMCC inventory used a literal PowerShell `*.ps1` path | 1 | Search containing directories with `-g '*.ps1'` filters instead; no source/build action was performed |
| Phase 21旧控制周期测试把控制量符号绑定到位置目标方向 | 1 | P降至0.8后残余速度可能主导内环；改为验证控制量与速度误差同号，并固定检查全部本轮PID值 |
| Phase 24首次主机回归仍断言正式首周期增力为1 us | 1 | 源码已正确直接输出150 us；更新正式测试契约并补充超速制动回归，中心0.5 us断言保留 |
| Phase 29首个组合补丁未匹配progress中的精确上下文 | 1 | 拆分为稳定锚点补丁；首次尝试未修改任何文件 |
| Phase 34首轮回归在无效帧清确认测试失败 | 1 | 10 ms确认门已到期后测试才送帧；改为到期前立即入队，生产状态机不变 |
| 首次从shell输出生成联合工程XML时混入命令包装文本 | 1 | 删除无效文件，明确从首个`<?xml`截取后再生成 |
| 首次通过`apply_patch`添加联合工程时补丁末尾多余换行 | 1 | 去除`*** End Patch`后的尾随换行并成功创建 |
| 首次29组回归在H456应用测试编译时命中通用stub类型冲突 | 1 | 将专用stub目录改为`-iquote`，确保其优先于公共`-Itests/stubs` |
