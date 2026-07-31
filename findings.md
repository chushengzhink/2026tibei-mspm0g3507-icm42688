# Findings & Decisions

## Phase 29 第三问恢复安全边界

- 三份1000 us正式脱困CSV均未进入`TO_MINUS_5`，在5000 ms停留于`TO_PLUS_5`并超时；位置峰值分别为+8.372/+12.392/+11.961 cm，速度峰值约14.57/21.49/21.94 cm/s。
- 三次脱困都实际请求并到达1000 us；释放后普通速度P项已经远超±200 us控制限幅，继续增加位置环或速度环增益不能抵消过强脱困冲量。
- 第2/3次在接近+12 cm时触发约220 ms失视回中，进一步证明当前首要问题是1000 us脱困边界，不是PID制动力不足。
- 用户明确撤销正式1000 us例外，恢复全路径1300–1700 us；本轮不修改P/D/I、观察器、序列判定、5秒门、Maix或遥测布局。
- 第三题按`O→+5 cm→-5 cm稳定`整段≤5秒验收；+5只需进入±1 cm范围并确认30 ms后折返，不要求低速，最终-5继续使用低速500 ms稳定门。

## Phase 30 第三问+5折返门修正

- 恢复1300 us后的三份CSV中，第1/3次已到达+5.322/+4.495 cm，但以8.12/6.20 cm/s进入允许带，旧共用低速门拒绝折返；第2次最高+3.419 cm。
- 用户选择保持1300–1700 us，不降低舵机下限；+5改为±1 cm内连续30 ms位置确认且不检查速度，最终-5仍要求±1 cm、≤1 cm/s连续500 ms。
- 本轮不修改位置/速度PID、脱困、观察器、5000 ms总超时、Maix、CSV或RAM布局，也不执行任何回归或构建检查。


## Phase 26 新球位置D增阻尼与端点低速确认
- 三份26列CSV均确认默认独立极限制动已关闭：活动段10 ms连续、视觉年龄最大60 ms、`brake_active=0`。
- 第1次最高+4.175 cm后振荡并超时；第3次进入+5允许区仅140 ms，速度峰值+7.71/-8.44 cm/s，同样超时。
- 第2次在4060 ms进入COMPLETE，但+5计时结束时速度约+5.59 cm/s，-5过程中最低-15.71 cm/s；完成后同时满足±1 cm和|速度|<=1 cm/s仅10 ms，随后在-6.88到-1.20 cm继续振荡。
- 当前端点稳定计时只检查位置，必须增加统一`|velocity|<=1.0 cm/s`门，任一条件越界即清除计时。
- 用户确认当前新球为后续正式基线，并选择只把位置D从0.20提高到0.40；P/I、速度环、直接150 us脱困、5 s、舵机边界和Maix均保持不变，D回退值为0.20。

## Phase 24 正式最大脱困与超速强制制动
- 两份100 us/s正式CSV均在5000 ms TIMEOUT；第一次3220 ms近似静止、第二次末段仅到98/150 us和1335 us，滚动本身不是主要耗时。
- 用户锁定正式正负阶段均在300 ms后直接施加150 us；中心调试仍使用50 us/s，关闭直接加力开关时回退到正式100 us/s。
- 用户锁定超速制动进入/退出差值为0.5/0.1 cm/s；+5制动目标1650 us、-5制动目标1300 us，制动期间冻结位置积分。
- CSV增加`brake_active`；可把脱困与制动活动位封装到现有`breakaway_active`字节，保持紧凑记录44字节和26400字节RAM区不变。
- 控制器的`ball_reset_pid_state()`已被停用、折返、失视回中、重捕获和目标重设共同调用；把制动标志纳入该函数即可覆盖全部安全清零路径。
- 超速判定必须在位置环首次计算目标速度后、积分提交前执行，使制动当周期即可冻结积分；最终控制输出阶段再用阶段对应边界覆盖普通PID。
- 现有正式测试固定期望首周期1 us和后续21 us，需要更新为直接150 us；中心0.5/10.5 us断言必须保持不变。
- 遥测记录尾部已有单字节`breakaway_active`，可改为flags位0=breakaway、位1=brake，`breakaway_fault`继续独立字节，结构尺寸不变。


## Phase 20 自动0 cm位置P外环
- 当前实现只切换自动控制模式：默认连续3帧视觉有效后目标设为0 cm并进入完整串级控制；PID数值保持位置1.6/0/0、速度25/0。
- 速度内环实测只证明球能减速停止且无持续往复；停在+2～+4 cm符合速度目标0的预期，不能据此声明中心精度或第3题性能达标。
- 两个交互式μVision窗口仍在运行；只能在现有project_ball窗口执行Rebuild，不启动新的UV4进程。

## Phase 21 位置P降档与UART0 CSV
- 实机确认位置符号正确且球反复过零；OLED `P+090`表示+9.0 cm，`U1700`表示舵机达到当前1700 us软件上限，因此本轮只把位置P从1.6降至0.8。
- UART0固定PA10/PA11、115200 8N1，可与UART2并存；杆球构建当前屏蔽UART0 ISR，需恢复但不得引入底盘遥测模块。
- 选择10 ms×600条、完整PID+视觉字段、PB24中止且舵机回到1500 us后才允许D/C；运行和回中期间不得阻塞导出。

## Requirements
- Phase 19杆球固件必须是独立工程；不得编译或初始化底盘、电机、编码器、LF04和IMU，PB24仍是唯一会触发杆球动作的按键。
- 杆球新增资源锁定为PB27/PINCM58/TIMA1_CCP1、UART2 PB15/PB16和TIMG6 1 ms；现有底盘工程与40%绝对PWM限制不得改变。
- 正式序列默认门控关闭，未实测的舵机方向、中位、像素端点和PID只可标成起点，不可写成已验证值。
- Confirmed physical LF04 order is PA31, PA12, PB8, PA27 from left to right, encoded as B1/B2/B4/B8.
- Right-outer LF04 is physically PA27 and must be GPIO input with pull-up.
- Race line assist uses four digital sensors and remains bounded by 500 mm/s wheel targets.
- UART0 at 115200 8N1 exports a standard CSV on single-byte `D`; `C` clears only while stopped.
- CSV must include elapsed/lap time, target/actual center speed, left/right PWM, and line bits.
- Braking samples must remain in RAM until the three-cycle stopped criterion completes.

## Research Findings
- 压缩包模块依赖仅为现有 `ml_pwm/ml_tim/ml_uart/ml_gpio/ml_board` 与标准库；不需要导入其SDK或生成配置。
- 压缩包控制器已有28字节帧解析、alpha-beta观测、1300–1700 us限幅和斜率限制，但正式序列缺少启动视觉门、丢球锁存中止、超时停控回中、一次上电启动限制和用时状态。
- 现有 `ml_pwm.c` 仅按TimerG初始化TIMG0/6/7/8/12；PB27已在GPIO枚举中定义为PINCM58，但PWM板级宏、资源所有者和TimerA初始化路径均不存在。
- 现有 `user/isr.c` 已提供TIMG6和UART2分发；TIMA1 PWM不需要中断处理。
- 现有 `chassis_key`只产生60 ms去抖后的按下事件，没有长按/释放事件；杆球应用需要在自身维护PB24长按状态，但可复用其短按防抖思想。
- 当前README/WIRING/ROBOT_SETUP仍只描述默认底盘和竞速工程；Phase 19文档应追加独立章节，避免改写已有赛道实测记录。
- 现有按键实测映射为PA14/PA15/PA24/PB24/PB25，低有效；杆球标定模式必须让方向键完全无舵机动作。
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

## Phase 18 Formal Three-Source Fusion Evidence
- The two latest attachment copies are byte-identical. The authoritative first monotonic session contains 187 rows from 100 to 18700 ms.
- The car stops at about 6164.1 mm and 374.99 degrees; the stop command occurs near 6127.9 mm at 172.1 mm/s, followed by about 36.2 mm of coast.
- The second straight already ends about 24 degrees ahead of the encoder route model. In the final 1000 mm, B0 occupies about 48.6%, recorded LF04 +120 mm/s occupies 65.7%, and at least one PWM reaches 20000 in 25.7% of samples.
- Current formal arbitration calculates saturated negative heading feedback but discards it whenever the LF04 residual has the opposite sign. On the curve, feedforward plus limited heading feedback still requests about +39.6 mm/s while stale B1/B0 consumes the full +120 mm/s cap.
- A read-only counterfactual replay using feedforward-first budget allocation, 120 ms B0 decay, 200+200 ms B1 taper, and 5-8 degree conflict attenuation reduces representative final biases to about +39.6 mm/s in long B0 and +52.5 mm/s in sustained conflicting B1.
- The existing velocity mode already closes fused yaw-rate feedback at `Kp=0.20`; Phase 18 must not add a duplicate IMU control loop.
- Current race link reports `RW-data=1040`, `ZI-data=29392`. Expanding 600 telemetry records from 44 to 52 bytes adds 4800 bytes and remains within device SRAM, subject to final link verification.

## Phase 18 Hardware Rejection and Corrected Arbitration
- The failed 27-column run contains 162 clean samples and ends at about 5997 mm because the car had already departed the line before completing the first semicircle.
- At 1749-1945 mm, B8 requests `-120 mm/s`, while route feedforward is about `+77` and heading feedback reaches `+37`; the rejected budget logic outputs `+65..+84 mm/s`, the opposite of the live LF04 recovery direction.
- By about 3019 mm, expected heading is 174.04 degrees but fused heading is only 137.86 degrees. First-curve B0 occupies about 66% and final steering is saturated for about 76% of samples.
- Sustained B15 begins near 4525 mm and persists through the remaining 38 samples; it is a consequence of the earlier departure, not the initiating defect.
- Restoring `22d672e` arbitration makes nonzero LF04 direction authoritative: opposite route/heading assistance is discarded, same-direction assistance may fill to the existing 90/120 mm/s cap, and B0 preserves the last confirmed recovery direction.
- Split route/heading/final telemetry, the 360 +/- 5 degree three-cycle finish gate, the 36 mm stop lead, and all lower safety limits remain.
- The public telemetry record remains 52 bytes and CSV remains 27 columns, but 600 RAM entries use an internal 44-byte representation to resolve the observed `0xC10` SRAM link overflow.

## Phase 22 Formal Ball Sequence Switch
- `BALL_BALANCE_ALLOW_SEQUENCE=1` is already enabled; the only configuration switch needed for the formal PB24 entry is `BALL_AUTO_CONTROL_MODE=BALL_AUTO_CONTROL_DISABLED`.
- In the current application, UART0 ball telemetry is compiled and initialized only for automatic calibration modes, so a config-only switch would lose the established CSV workflow.
- The disabled/formal branch currently refreshes four software-I2C OLED lines every 100 ms during active control, which would reintroduce the previously measured approximately 80 ms PID stalls.
- PB24 currently aborts only `TO_PLUS_5`/`TO_MINUS_5`; after `COMPLETE` the controller intentionally remains enabled at -5 cm and the application has no PB24 path to recenter.
- Core breakaway assistance is already isolated from scoring by requiring `sequence_state==IDLE` and target 0 cm; formal sequence behavior can remain unchanged.
- No controller API addition is required for completed-sequence recentering: `ball_balance_enable(false)` already disables the completed cascade while preserving `BALL_SEQUENCE_COMPLETE` and its elapsed time.
# Phase 23 正式脱困斜坡与反向抗抖

- 两份1300 us固件CSV均以5000 ms TIMEOUT结束，最低实际脉宽分别为1329/1327 us，未到1300 us边界，不能继续降低全局下限。
- 第一份补偿被像素速度抖动切成4段；第二份在+2.15 cm附近卡住时增力仅到107/150 us即超时。
- 本轮只加速正式序列斜坡到100 us/s，并要求反向速度与相对触发起点背离至少0.15 cm同时成立才清除补偿；中心模式、PID、Maix和CSV不变。
- 当前控制器在补偿已激活时仅凭“非朝目标且非静止”立即清除，正是单像素反向速度尖峰会切断斜坡的代码路径。
- 已有锁存字段`breakaway_start_position_cm`和`breakaway_direction`足以计算目标方向位移/速度，无需扩展状态或RAM。
- 现有中心像素抖动测试必须继续保持50 us/s断言；正式序列测试应单独断言100 us/s，避免改动中心已验证行为。
- 控制周期固定10 ms，因此正式100 us/s对应每次控制增力1.0 us，中心50 us/s仍为0.5 us。
- `ball_update_breakaway()`在PID积分更新前执行，其返回值直接冻结位置积分；抗抖分支只要继续返回true即可保持积分冻结。
- 状态接口导出的`breakaway_boost_us`已包含锁存方向，测试可直接比较正式+5为正、中心正侧回0为负，无需新增遥测字段。
- 文档当前已正确锁定正式1300～1650 us、中心1400～1650 us，但仍把正式斜坡写为50 us/s；需要只更新“当前策略”段落，保留历史CSV段落中的旧参数以维持实验可追溯性。
- 源码差异确认没有新增状态字段或公开接口，只有两个配置宏、阶段感知斜坡函数和激活态反向双门逻辑；因此CSV 25列和44字节记录布局不受影响。
- `user/Objects/ball/project_ball.build_log.htm`确认当前工具链为ARM Compiler 5.06 update 5，且交互式project_ball构建目录存在；本轮只做单文件编译，不能启动并行UV4全量构建。
- 完整主机回归为`25/25 passed`；ARMCC正式补偿启用/禁用配置均无诊断；project_ball的38个源文件通过隔离审计，未包含底盘、电机、编码器、LF04或IMU模块；`git diff --check`通过。
## 2026-07-31 正式序列全程极限制动复测

- 三份26列CSV活动段均连续10 ms，视觉年龄最大60 ms，无失视；最大位置仅为`+3.421/+2.997/+3.314 cm`，全部停留在`TO_PLUS_5`并于5000 ms超时。
- 独立制动分别在位置约`+0.15～+0.92 cm`便触发，直接请求1650 us并保持约0.52～0.58 s；球仅到`+3.0～+3.4 cm`就反向到`-0.73～-1.36 cm`，形成约1.5 s周期的1300/1650 us极限往返。
- 三次直接150 us脱困均已克服起点静摩擦；故障不是力度、视觉或控制方向，而是独立制动重复速度内环且触发过早。
- 用户选择默认关闭独立极限制动；保留直接最大脱困、普通速度环连续制动、26列CSV及旧制动配置作为诊断回退。
- 当前独立制动由`BALL_SEQUENCE_OVERSPEED_BRAKE_ENABLED`完整包围，默认值可安全改为0而无需删除实现；`brake_active`在禁用路径会被主动清零。
- 现有主机测试只覆盖开关为1的0.5/0.1 cm/s迟滞，需要增加默认关闭回归，并用宏覆盖值1另跑同一测试保持诊断回退路径覆盖。
- README、ROBOT_SETUP和杆球指南仍停留在100 us/s渐增及25列CSV描述，必须一并收口为正式直接150 us、独立极限制动默认关闭及26列CSV。
# Phase 27 audit findings

- 用户锁定速度环P/D=27/3，不允许回退到旧25/0；正式运行必须保持串级模式。
- 当前生产配置被手动改成位置P/D/I=1.6/1.0/0、序列下限750 us和超时50000 ms；测试与文档仍保留旧断言，完整主机回归因此为24/25。
- 现有最终控制量统一限制为±200 us，因此750 us配置实际不可达，脱困故障也无法按750 us边界触发。
- 当时用户选择位置P/D/I恢复0.8/0.40/0.30、超时恢复5000 ms，并临时允许正式+5脱困到1000 us；该1000 us例外已在Phase 29根据实测过冲撤销，当前全路径恢复1300～1700 us。
- 现有正式启动、目标折返和连续3帧重捕获路径都已设置BALL_CONTROL_CASCADE，只需增加回归锁定而无需重写入口。

# Phase 28 empty CSV findings

- 实机D命令只返回完整26列表头而无数据行；导出器当前无条件先发表头，`g_record_count==0`时会静默形成该现象。
- 当前应用仅在PB24启动分支返回OK时调用`ball_telemetry_session_start()`，实际序列状态与遥测生命周期没有集中绑定；应用测试还用遥测桩替代真实记录区，不能复现空表头。
- 当前AXF符号确认`g_records=0x20200470..0x20206b8f`共26400字节，`g_record_count=0x20200008`，无重叠或第二RAM问题。
- 修复必须保持串级P/D/I=0.8/0.40/0.30、速度P/D=27/3、5000 ms、26列和44字节记录布局不变。
- 原脱困负方向安全清零测试在舵机到达1400 us后仅继续300 ms，却要求增力至少95 us；当前中心斜坡50 us/s只保证增力继续为正且不超过150 us。该断言与500 ms故障门无关，改为验证有效范围而不改生产逻辑。
- 当前完整主机回归扩展为26组并全部通过；新增联合测试实际链接生产应用和遥测模块，导出行数等于记录数+表头、每行25个逗号、重复D字节一致。
- 交互式UV4仍打开`project_ball.uvprojx`；只能进行独立ARMCC单编，不能并行启动UV4全量构建。
- 用户已在现有μVision窗口完成新Rebuild，构建日志时间为2026-07-31 18:24:01；新AXF仍将26400字节`g_records`放在0x20200470，计数器位于0x20200008，无布局漂移。
# 2026-08-01 Phase 31 初始约束

- 用户提供三份最新第三题26列CSV，现象为+5端容易略超，-5端多数能接近但容易在约-4 cm停住。
- 本轮目标不是继续单点调一个阈值，而是检查正式序列是否缺少完整的负端捕获和静止恢复状态逻辑。
- 用户明确要求本轮不做回归、编译、`git diff --check`或Keil验证；只做数据诊断、源码修正和必要文档同步。
- 安全边界继续保持舵机1300–1700 us、PB24运行中立即中止、视觉150 ms安全门及5000 ms正式总门。
- 根目录实际`AGENTS.md`记录的当前基线已经包含负端低速捕获带±0.4 cm，以及捕获后`3×位置误差-1×实际速度`、限幅±2 cm/s的参考速度；不能把本轮问题简单归因于“尚未加入阻尼”。
- 前一轮实测的共同现象是负端最低可达约-4.66至-5.53 cm，但最终回弹到-3至-4 cm；本轮CSV需重点区分捕获未发生、捕获后退出、端点脱困禁用和5秒预算耗尽四类原因。
- `README.md`、`WIRING.md`和`ROBOT_SETUP.md`已完整核对；接线与安全边界没有要求变化，代码修改范围应限于杆球控制器及必要说明。
- 当前书面状态机为：负向5 cm/s巡航 -> 按停车距离锁存制动 -> `2×误差`首次接近（限4 cm/s） -> 中心±0.4 cm且低速时捕获 -> `3×误差-1×实际速度`保持（限±2 cm/s） -> 题面±1 cm且低速连续500 ms完成。
- 文档存在正向制动估计18/20 cm/s²的内部不一致，证明历史说明已有漂移；本轮判断必须以当前源码常量和CSV实际行为为准。

## 三份最新CSV初步结论

- 三份文件SHA均不同，分别为469/461/495条；活动段几乎全部10 ms连续，仅每份终态各有一个75 ms间隔。
- 三轮都正常从`sequence_state=1`切到`2`，+5折返位置为+4.783/+4.608/+4.748 cm；随后都未进入最终±1 cm低速合格带。
- 负端最低位置仅-3.385/-3.689/-3.499 cm；在结束前目标速度仍为-5.00 cm/s，说明当前负端捕获和捕获后阻尼逻辑三次均未获得执行机会。
- 三轮最终都在约-3.33/-3.66/-3.50 cm触发`breakaway_active=1`、`boost=-150 us`、舵机1650 us；保持后序列提前转为状态5，而非5000 ms正常超时。
- 因此主要缺陷不是捕获带或捕获后阻尼参数，而是负端静止恢复仍采用旧的“满增力到边界并按固定时间锁存故障”路径；它在球距离目标仍1.3–1.7 cm时直接终止整次序列。
- `near -4 stationary`样本统计为0，是因为三轮都在更靠正侧的-3.3至-3.7 cm卡住；用户的“约-4”是符合视觉精度的现象描述。
- 相关生产代码集中在`code/ball_balance.c`、`code/ball_balance.h`、`code/ball_balance_config.h`、`code/ball_balance_app.c`和`code/ball_telemetry.c`；核心修正应优先限制在控制器与配置，不改遥测布局和应用安全入口。
- 当前源码常量确认正向制动估计实际为18 cm/s²，负向为24 cm/s²；README末尾的“正向20”属于过期说明。
- 当前负向脱困100 ms即启动、立即施加150 us，1650 us边界保持500 ms后会触发全局`breakaway_fault`并把正式序列置为`ABORTED`；CSV状态5正是`BALL_SEQUENCE_ABORTED`。
- 状态结构只有一个通用脱困活动/第二阶段/故障状态，没有专门的负端恢复阶段或尝试预算，这与CSV显示的“一次满增力后提前终止”一致。
- `ball_sequence_minus_velocity_reference()`只有在一次性`sequence_approach_braking`锁存后才从固定-5 cm/s巡航切到按误差收敛；CSV末尾仍为-5，说明三轮卡住前均未锁存该制动状态。
- `ball_update_breakaway()`与轨迹制动状态彼此独立：负端误差>1 cm且100 ms内进度不足0.15 cm即可直接激活150 us，1650 us保持500 ms则调用`ball_trip_breakaway_fault()`，将序列ABORT并回中。
- 捕获态只有位置±0.4 cm且速度≤1 cm/s才设置；其本身不退出。因此本轮不是“捕获后退出”，而是完全未捕获。
- 负向轨迹逐100 ms重放显示每轮都有两次停滞：第一次约在-1.2至-2.1 cm，1650 us脱困后释放；第二次约在-3.3至-3.7 cm，1650 us保持500 ms后ABORT。
- 巡航阶段把反向制动控制限制为最多+20 us（舵机不低于1480 us），实际速度从约-10 cm/s自然降至0时，停车距离条件始终没有命中，因此`sequence_approach_braking`三轮均保持false。
- 当前存在明确死区：未捕获要求进入中心±0.4 cm，但脱困在负端只对误差>1 cm启用；停在-4.0至-4.6 cm时既不能捕获也不能脱困，而最终完成门却只检查±1 cm/低速，可能在未捕获状态直接完成。
- 修正方案锁定为：负端未捕获时脱困阈值使用±0.4 cm；1650 us阶段保持250 ms后升级到现有绝对上限1700 us再保持250 ms；最终500 ms完成累计必须要求`sequence_endpoint_captured=true`；+5确认缩短到10 ms以减少高速度穿带带来的额外超程。
- 1700 us不是新扩限：当前全局绝对上限一直为1700 us，三份CSV在+5折返后的反向制动阶段均已实际请求并到达1700 us。本轮只让负端静止恢复第二阶段使用同一现有上限。
- 最终实现同时修复四个互相关联的缺口：负向停车距离门可能永不命中、±0.4至±1.0 cm恢复死区、未捕获即可累计完成、1650 us单级恢复在固定500 ms后直接ABORT。
- 修改后负向流程为：巡航 -> 停车距离或2.0 cm固定接近区锁存 -> 2×误差接近 -> 必要时1650/1700 us两级恢复 -> ±0.4 cm低速捕获 -> `3×误差-1×速度`保持 -> 已捕获前提下±1 cm低速500 ms完成。
- 按用户明确要求，本轮没有运行主机回归、编译、静态检查、差异检查或Keil构建；结果必须以重新Rebuild/Download后的新CSV判断。

# 2026-08-01 Phase 32 双端轨迹能量收敛

- 新三轮均为501条、500个连续10 ms间隔，最终均为`TIMEOUT`而非`ABORTED`，证明上一轮负端1650/1700 us两级恢复已按设计工作。
- +5折返发生在+4.221/+4.376/+4.557 cm，但折返速度仍为6.78/8.81/10.29 cm/s，惯性峰值达到+5.814/+6.344/+7.029 cm；10 ms确认门已不是主要瓶颈。
- 正向首次进入接近控制时仍在约+3.34/+3.37/+3.57 cm且速度6.64～8.56 cm/s，现有4.5 cm/s巡航和2倍接近参考无法在+5带前消除动能。
- 负端最低位置为-3.866/-4.322/-4.333 cm；第2/3轮在题面±1 cm且低速带内最长保持140/100 ms，随后因接近参考制动过强回弹至-3.x。
- 用户锁定题面优先：先在-5±1 cm且低速完成500 ms，再由`COMPLETE`后的普通带积分位置环继续向-5中心收敛。
- 用户锁定正向3.0 cm/s巡航和2.5 cm早接近；测试源码同步更新，但不执行回归、编译、差异检查或Keil构建。
- `tests/ball_balance_test.c`仍含上一轮未同步的4.5 cm/s、负向2倍、±0.4 cm和+5确认30 ms硬断言；本轮必须一并更新，避免后续测试契约继续描述旧固件。
- 现有负端测试已覆盖高速穿越、低速进入、捕获后回弹和再恢复，可在原函数内收口为“高速穿带不捕获、低速进入±1 cm捕获、捕获后才允许完成”，不新增平行测试框架。
- 最终生产参数为：正向3.0 cm/s巡航、2.5 cm接近区、1.2倍位置参考；负向5.0 cm/s巡航、2.0 cm接近区、3倍位置参考、题面±1 cm低速捕获。
- `COMPLETE`后控制模式和-5 cm目标保持不变，序列状态退出后自动回到普通带积分位置环，因此可以先锁定题面合格用时，再继续向-5中心收敛，无需新增保持状态或公共接口。
- 测试源码已同步参数断言、10 ms确认恢复、正向固定接近区和负向低速捕获/阻尼场景；按用户要求未执行。

# 2026-08-01 Phase 33 机械水平零点

- 用户最终复核确认1500 us并非杆面水平位置，1525 us为闭环水平零点。
- 本轮CSV为452条有效记录，+5总体峰值已降到约+5.261 cm，但负端最低仅-3.330 cm，并在4480 ms以状态5`ABORTED`结束。
- 最后一次负端脱困在3940～4470 ms持续540 ms，位置从-3.185到-3.330 cm、末速度-0.74 cm/s；球仍朝目标运动，但0.145 cm位移比0.15 cm释放门少0.005 cm，因此仍累计满边界故障门。
- 正确边界是分离两个概念：`BALL_SERVO_CENTER_US=1500`继续用于人工标定、急停、失视、故障和导出回中；`BALL_CONTROL_NEUTRAL_US=1525`用于正常待机与全部闭环控制换算。
- 正式脱困1300/1650/1700 us必须继续作为绝对脉宽；公共API、Maix协议、CSV列和状态枚举不变。
- 按用户要求，本轮同步测试源码但不运行回归、编译、差异检查或Keil构建。

# 2026-08-01 Phase 34 H4/H5/H6融合约束

- 用户最终确认闭环水平零点为1525 us，1500 us只保留为安全/人工/导出回中；当前源码已是1525 us，但测试、README、ROBOT_SETUP和Phase 33记录仍残留1550 us。
- 当前主机回归前20组通过，第21组`ball_balance_test`因硬断言1550 us失败；两扇交互式UV4窗口分别打开`project_ball`和`project_track`，禁止并行启动命令行UV4。
- 现有底盘与杆球遥测各有600×44=26400字节静态缓冲，不能同时链接进唯一32 KB SRAM；联合工程必须使用单独40字节×600条缓冲，并把兼容底盘遥测容量缩到最小。
- 用户锁定单固件H4/H5/H6选择、通过B/A后自动平滑停车、运行期OLED冻结、240 mm/s巡航和150 mm/s²加减速。
- H6目标由左右键在±10 cm内按0.5 cm调整；任一10 ms样本超过±1 cm锁存不合格，连续100 ms超过±2 cm停车。
- H4按1500 mm通过B；H5/H6沿用当前5932 mm实测A点参考和355°～365°连续3周期航向门，通过时冻结评分时间与评分段最大球误差。
- H456任务层已独立于第2问停车/回正状态机：通过评分点后将路线前馈与航向反馈清零，由LF04在150 mm/s²减速期间继续循迹；第2问5932 mm停车逻辑未改。
- 任务层按“先判通过、再判超时”处理同一20 ms周期，恰好在8/30秒通过仍合格；超时未通过会进入受控减速，缺失整圈航向门且超过5982 mm则立即锁存赛道故障。
- 联合工程必须保持`BALL_BALANCE_BUILD=0`语义：`user/isr.c`据此保留TIMG0/7/8/12、UART1/3和GROUP1底盘中断；UART2视觉中断无条件保留。
- `project_h456.uvprojx`已从竞速工程源配置派生，只定义`H456_COMBINED_BUILD=1,BALL_BALANCE_ALLOW_SEQUENCE=0,CHASSIS_TELEMETRY_CAPACITY=1`；不链接`chassis_track_app`、`ball_balance_app`或`ball_telemetry`。
- 联合工程保留`chassis_telemetry.c`的一条兼容记录，同时只含一个`h456_telemetry.c`的24000字节主缓冲；XML可由PowerShell正常解析。
- 应用原本在任务层判定通过后的下一轮询才设置`score_frozen`，会把通过后一个样本计入滚球最大误差；冻结现与通过判定处于同一20 ms周期。
- 最终便携验证为主机`29/29 passed`；联合工程清单内39个C源在ARMCC 5.06和联合宏下逐文件0失败、0诊断，冻结修正后的`h456_app.c`再次单编通过。
- 默认、竞速、滚球、联合四个工程XML均可解析、源路径缺失数均为0；联合源清单中不存在`chassis_track_app.c`、`ball_balance_app.c`或`ball_telemetry.c`。
- 安全审计确认舵机1300/1500/1525/1700 us语义、20000 PWM硬限、8周期堵转、240 mm/s、150 mm/s²、8/30秒门和600×40字节联合缓冲保持正确；联合源码没有调用第三问序列入口。

# 2026-08-01 Phase 35 Q3地形学习与OLED重写约束

- 用户明确要求实现已确认方案，并追加“Q3 OLED应用层也重新写”，不再保留现有Q3页面状态机。
- 允许复用`project_ball_q3`工程外壳、底层软件I2C/OLED驱动、Maix 32字节协议、PB24、UART0和舵机驱动；现有Q3控制算法与OLED应用契约均可替换。
- Q3反馈只能来自MaixCam小球位置/时间戳，执行器只有舵机；不得链接IMU、编码器、LF04、底盘或旧`ball_balance`控制器。
- 舵机绝对范围固定1300--1700 us，PB24仍是正式运动唯一启动确认及运行中立即急停。
- 方案锁定为离线全杆LUT、上电O点微调、+5合法带即时折返、-5低速稳定、卡滞反摇救援；机械标定后固定不拆。
- 现有Q3草案已包含512×40字节正式遥测、32字节Maix协议和独立Keil工程，可作为外围与内存基线，但其中心短试探没有完整地形学习与主动回O能力。
- 当前工作区含大量用户/既有未提交H456、H5、Q3和I2C/OLED修改；本阶段不得回滚、覆盖或清理不相关文件。
- 现有Q3核心状态枚举把预标定、正式轨迹和故障混在单层状态机中；离线全杆标定没有公开入口或数据结构，需要重构公开状态而不能只增几个参数。
- 当前Q3正式遥测记录恰为40字节、容量512，现有CSV 20列；可以通过替换正负死区/CRC等低价值字段加入LUT索引、启动缩放、预测停止点、救援级别和进展，保持20480字节主缓冲不变。
- 现有Q3 OLED应用约640行，初始化、页面渲染、UART命令和遥测会话耦合较重；用户已授权重写，适合拆成小型页面模型和缓存写入服务，但继续复用`OLED_Init/OLED_ShowLine`底层。
- 现有Q3正式运行只写OLED首行、非运行100 ms缓存刷新，这一不阻塞原则应保留；新的页面需覆盖WAIT/PRECAL/READY、MAP确认/扫描、正式四阶段、救援、故障和CSV状态。
- 当前`tests/run_host_tests.ps1`只纳入`q3_ball_app_test`，没有运行现存`q3_ball_test.c`或Q3遥测测试；本阶段必须把Q3核心、profile、telemetry、OLED应用测试全部纳入完整回归。
- 现有Q3预标定仅在O点施加固定±60 us/120 ms，并根据不足0.15 cm响应把死区设为45 us；它既不生成分位置特性，也不能主动纠正停在O外的静止球，必须整体替换。
- 新Q3正式记录已在40字节内重排为22列：保留时间/状态/位置/速度/目标/舵机/视觉/原始像素，并加入响应缩放、预测停止点、停滞进展、LUT格和救援阶段/次数；容量仍为512条即20480字节。
- 新OLED应用不再暴露UART直接手动脉宽；UART的`K/L/A/W/D/C`只能武装标定、导出或选模式，正式运动和地图运动仍必须由PB24确认。
- 安全语义重新明确：1500 us为WAIT、ABORT、TIMEOUT、失视、profile/标定故障和地图完成后的安全中位；1525 us及开机修正值只用于READY/闭环局部平衡。
- 完整主机回归已扩为36组并全部通过，新增Q3 profile、核心闭环、遥测、OLED应用和Python profile工具测试；原底盘、旧杆球、H5/H456均未回归。
- 最终Q3构建在OLED启动页预写运行页修正后为`Code=21252, RO-data=2876, RW-data=888, ZI-data=22408`，`0 Error(s), 0 Warning(s)`；Map唯一`RW_IRAM2=0x5b00/0x8000`，`g_records=20480`。
