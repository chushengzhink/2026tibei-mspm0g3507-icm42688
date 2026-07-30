# MSPM0G3507 通用双轮差速底盘模板

本工程面向立创·天猛星 MSPM0G3507 开发板、TB6612 双路电机驱动和两只 MG513X 霍尔编码器电机。

SW6五向键映射已在实物上确认：上/左/下/中/右依次对应PA14/PA15/PA24/PB24/PB25。默认固件恢复完整底盘自检，任何运动都必须由垂直中键确认；电气检查见 [WIRING.md](WIRING.md)，软件与标定原理见 [ROBOT_SETUP.md](ROBOT_SETUP.md)，现场逐项验收和数据记录使用 [HARDWARE_ACCEPTANCE.md](HARDWARE_ACCEPTANCE.md)。

## 默认能力

- 左右编码器均由四根 GPIO 双边沿中断完成软件 4× 解码。
- 每个车轮使用独立 PID、前馈、轮径标定和方向配置。
- 20 ms 轮速闭环，计数速度使用系数 0.35 的一阶低通滤波。
- 编码器低频约束与ICM42688短时角速度互补融合，输出编码器航向、融合航向和融合角速度。
- 差速中点积分里程计使用融合航向输出 `x_mm`、`y_mm`，同时保留累计距离和累计 tick。
- 提供轮速、线速度/角速度、定距、原地转向和圆弧非阻塞接口。
- 定距、转向和圆弧使用加减速轨迹；完成条件需连续三个周期进入 2 mm 或 1°窗口。
- 8 个控制周期无编码器反馈触发堵转保护，约 160 ms。
- 电机驱动绝对PWM硬限制为20000/50000，即40%；默认安全自检仍使用`PID 11500 + 前馈6000`，输出不超过原35%。
- LF04通用读取仍不参与默认自检；独立竞速工程由LF04决定横向转向方向，编码器负责里程/赛段，编码器+IMU融合只提供不能抵消或反转红外方向的辅助修正。
- 直线、转向、圆弧和速度模式在运动层使用融合航向/角速度纠偏；单轮PID仍只使用编码器。
- IMU读错或超过100 ms未更新时无跳变退化为编码器航向。
- 10 Hz RAM遥测可保存600条，每条44字节；记录累计航向、速度、实际PWM和LF04可用/恢复/异常状态，并可在停车后通过UART0导出20列CSV。
- 断开12 V后可在开机前按住中键进入OLED滚动标定页，无需连接电脑即可读取左右相对tick和BAD增量。

MG513X默认采用13 PPR、1:28减速比、65 mm轮径：

```text
13 × 28 × 4 = 1456 tick/轮
π × 65 / 1456 = 0.1402497 mm/tick
```

理论值只用于首次启动。左右轮必须分别按实车滚动距离标定。

当前一组正反各1 m滚动样本得到暂定值：左轮 `0.1413727 mm/tick`，右轮 `0.1434926 mm/tick`。它们已写入默认配置，但仍需通过500 mm/1 m实车回归确认最终距离精度。

## OLED滚动标定入口

断开12 V，只保留5 V逻辑供电；开机前按住垂直中键并保持车体静止。IMU校准完成后显示 `ROLL C=ZERO`，第二、三行是左右有符号相对tick，第四行是相对BAD。首次进入自动建立零基线，之后每次中键有效按压只清零显示基线，不启动电机。断电后不按中键重启即可返回普通自检。详细记录方法见 [HARDWARE_ACCEPTANCE.md](HARDWARE_ACCEPTANCE.md)。

## 软件分层

```text
chassis_self_test / 用户任务
        |
        +-- chassis：非阻塞公共API、20 ms调度、遥测
        |      +-- chassis_motion：轨迹、融合航向/角速度反馈、堵转与急停
        |      +-- chassis_heading_fusion：编码器低频约束与陀螺仪短时预测
        |      +-- chassis_odometry：融合航向中点积分并保留编码器航向
        |      +-- motor_velocity：左右轮PID、前馈、速度滤波
        |
        +-- ml_encoder + ml_quadrature：GPIO软件4×解码
        +-- ml_motor_driver：TB6612方向/PWM和40%绝对硬限幅
        +-- icm42688_service：姿态、映射后零偏校正角速度和健康状态
        +-- line_sensor + chassis_track_line_control：LF04两路三级P控制与方向优先仲裁
        +-- chassis_track_mission：编码器+IMU胶囊赛道主任务
```

常用入口在 `code/chassis.h`：

```c
chassis_init(&g_chassis_default_config);
chassis_set_wheel_speed(100.0f, 100.0f);
chassis_set_velocity(120.0f, 0.0f);
chassis_move_mm(500.0f, 120.0f);
chassis_rotate_deg(90.0f, 60.0f);
chassis_arc(300.0f, 180.0f, 100.0f);

while (1) {
    chassis_poll();
}
```

所有命令均为非阻塞调用。应用应持续执行 `chassis_poll()`，并通过 `chassis_get_status()`检查运行、完成或故障状态。

## 目录

- `ml_libs/`：板级GPIO、定时器、UART、I²C、电机、编码器和传感器驱动。
- `code/`：PID、底盘配置、里程计、运动控制、遥测、自检和可选循迹。
- `tests/`：可在PC上运行的纯算法测试。
- `examples/legacy_drawing_task/`：已从默认工程移除的历史任务代码，仅供参考。
- `examples/attitude_monitor/`：已从默认工程移除的独立姿态显示示例。
- `user/project.uvprojx`：默认Keil底盘自检工程。
- `user/project_track.uvprojx`：独立竞速工程，定义`CHASSIS_TRACK_MISSION_BUILD=1`。

## 构建

运行 PC 主机测试（需要 `gcc`）：

```text
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_host_tests.ps1
```

主机测试只验证可移植算法和安全状态机，不能替代 ARMCC 全量构建或实车验收。

Keil 全量构建：

```text
D:\Keil_v5\UV4\UV4.exe -b user\project.uvprojx -j0
```

竞速工程不要在默认架空验收前烧录；验收完成后在现有μVision窗口打开并Rebuild：

```text
user\project_track.uvprojx
```

同一竞速固件还提供独立的纯LF04诊断入口：上电前按住上键，完成白底与IMU校准后松开上键；进入`LF ONLY READY`后用上/下键选择60、120或200 mm/s。READY或正常完成页不检查当前B0-B15位型，按中键即可启动；若从B0启动则立即开始丢线计时。转向只取LF04输出，IMU只记录，编码器仍负责左右轮闭环、里程和堵转；每次运行1000 mm自动停车。`B0`进入120 mm/s限速短搜，持续300 ms显示`LF LOST STOP`并要求重新上电；所有`B1-B15`均为正常可用位型。正常完成可导出CSV、重新摆车并选择下一档再次运行。

竞速任务按中键边沿把当前位置和航向定义为A点零点并顺时针启动，路线为1.5 m直线、R500 mm半圆、1.5 m直线、R500 mm半圆，总长6141.6 mm。基线直线/弯道均为360 mm/s；通过实车验收后可用`CHASSIS_TRACK_SPEED_STAGE=1/2`把直线依次提高到380/400 mm/s，弯道保持360 mm/s。加减速度400 mm/s²，末段降至100 mm/s。

编码器里程生成AB/BC/CD/DA的期望航向，融合航向误差以`Kp=4.0`和`±0.35 rad/s`限幅修正，现有速度模式继续提供融合角速度反馈。终点必须同时满足编码器中心里程达到`6141.6-15 mm`和累计融合顺时针航向至少350°；到`route+50 mm`仍未同时满足则锁停并显示`FAULT LAP CHECK`。

LF04从左到右为PA31/PA12/PB8/PA27。按“从车头看向车尾”，左侧PA31/PA12合并为`black_bits & 0x03`，右侧PB8/PA27合并为`black_bits & 0x0C`：两组都有线时居中；仅左组有线时必须左轮减速、右轮加速，仅右组有线时必须左轮加速、右轮减速。所有`B1-B15`均可用，只有`B0`表示丢线。控制器每20 ms按左/居中/右三态执行`Kp=1.0、Ki=0、Kd=0`的P控制，不计算探头质心或方向防抖；红外差速为当前中心速度的22%且绝对值不超过90 mm/s。正式竞速中编码器/IMU轮间偏置只在与红外同向时叠加，总偏置仍限制在90 mm/s，反向辅助直接丢弃；双组居中时保留路线融合修正。`B0`把中心速度限制到120 mm/s并沿最后一次可靠红外方向短搜，辅助修正不得反转短搜方向；若此前居中则只跟随路线方向，持续300 ms锁停。红外不参与终点双门，运行中中键仍立即急停。

GPIO输入初始化会先清除遗留输出使能；竞速初始化、启动边沿和每次20 ms采样都会把四路LF04重新确认为普通GPIO上拉输入。LF04固定使用白底高、黑线低逻辑，期望白底为`WF`，归一化采用`(~R)&0x0F`，不再把开机瞬态保存为极性基线。IMU校准期间还必须连续10个20 ms样本得到`RF`才允许进入READY；否则保持停车并显示`LF WHITE WAIT`。启动前重申失败会禁止启动，运行中失败会立即锁停并显示`LF GPIO FAULT`。RUN第四行每秒交替显示`Rr Ww Bb`与`LFb Ln DnHn`，分别用于核对原始电平/固定白底期望/归一化黑线位，以及丢线恢复/距离门/航向门。

UART0固定为PA10/PA11、115200、8N1、无流控。竞速完全停稳后发送单个`D`导出纯20列CSV，确认文件保存后再发送`C`清空RAM；运行和制动期间两者均不执行，仅每秒最多回复一次`BUSY`。编码器和融合航向为不回绕的累计值，原始IMU Yaw保留回绕诊断；同时间戳立即采样覆盖上一条，导出的时间戳严格递增。SSCOM必须关闭“发→/收←”方向标记、时间标签、定时发送和自动换行，只发一次ASCII `D`或十六进制`44`并等待完整导出。

验收标准为 `0 Error(s), 0 Warning(s)`。烧录和上电前必须先阅读 `WIRING.md`，实车自检和标定步骤见 `ROBOT_SETUP.md`。

Fusion算法授权保留在 `ml_libs/FUSION_LICENSE.md`，硬件规格书和原始资料均未从仓库删除。
