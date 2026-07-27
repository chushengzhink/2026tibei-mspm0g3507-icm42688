# 天猛星 MSPM0G3507 激光绘图小车
本工程面向立创天猛星 TI MSPM0G3507 核心板及配套扩展板，使用 ARM Compiler 5.06 和手动外设初始化。当前固件已经接入 MAIXCAM PRO 数字识别、串口屏按钮启动、LF04 四路红外循迹、双电机编码器闭环、差速绘圆、OLED 状态显示、中键启动/急停和返回停车区；不使用 MPU6050，也不控制外部激光电源。逐脚接线和安全上电顺序见 `WIRING.md`，状态机、标定步骤和故障码见 `ROBOT_SETUP.md`。

> 当前 `user/main.c` 是 ICM42688 姿态解算独立演示入口，不启动小车任务状态机；小车任务模块仍保留在工程中，后续可通过 `icm42688_service.h` 接入，但本次不参与运动控制。

## 启动与当前入口

启动链如下：

```text
Reset_Handler -> ARMCC __main -> main -> system_init
              -> ICM42688 service -> gyro calibration -> attitude loop
```

- `system_init()` 复位并使能 GPIOA/GPIOB，使用 SYSOSC 驱动 SYSPLL，将 MCLK 配置为 80 MHz。
- 当前入口初始化 OLED 和 ICM42688 姿态服务；服务内部管理传感器、姿态算法、TIMG8 毫秒时基与静止校准，完成后持续输出 Pitch/Roll/Yaw。
- `robot_mission_init()` 及原有小车任务代码仍参与编译，但当前 `main()` 不调用它；恢复任务入口时需自行决定姿态数据如何接入运动控制。
- LED1 阳极经 1 kΩ 限流电阻连接 3.3 V，阴极连接 PB14，因此 PB14 低电平点亮、高电平熄灭。
- 核心板 LED 位于 PB22，但 PB22 同时连接 `IMU_MOSI`，因此不作为默认 LED。
- `ti_msp_dl_config.c/.h` 和 `empty.syscfg` 仅作参考，不参与 Keil 工程编译，也不得与 `system_init()` 混用。

## 已接入驱动的引脚

| 功能 | 引脚 | 驱动说明 |
| --- | --- | --- |
| LED1 | PB14 / PINCM31 | 默认 LED，低电平有效 |
| LED2 | PB18 / PINCM44 | 低电平有效 |
| LED3 | PA22 / PINCM47 | 低电平有效 |
| 核心板 LED | PB22 / PINCM50 | 低电平有效，与 IMU_MOSI 冲突 |
| UART0 | PA10 TX、PA11 RX | 默认 `printf` 目标，4 MHz MFCLK |
| UART1 | PA8 TX、PA9 RX | 串口屏，115200、8N1 |
| UART2 | PB15 TX、PB16 RX | MAIXCAM，115200、8N1 |
| UART3 | PA26 TX、PB13 RX | PA26 与 TIMG8_CH0 冲突 |
| OLED 软件 I2C | PB2 SCL、PB3 SDA | SSD1306，7 位地址 0x3C |
| ICM42688 软件 I2C | PA1 SCL、PA0 SDA | 7 位地址 0x68，AD0 接地 |
| 电机 A | PA28 PWM、PA13/PB26 方向 | TIMG7_CH0 |
| 电机 B | PB20 PWM、PB9/PB7 方向 | TIMG12_CH0 |
| 编码器 1 | PB23 A、PB12 B | A 相下降沿中断 |
| 编码器 2 | PB4 A、PB5 B | A 相下降沿中断 |
| 加热 PWM | PB19 / TIMG7_CH1 | 仅登记通用 PWM 路由 |

## ICM42688 姿态演示

- 上电后保持模块静止约 3 秒。程序连续收集 300 个 100 Hz 样本，检测到移动会清零进度并重新校准。
- 校准只估计三轴陀螺仪零偏；Pitch/Roll 使用平均重力方向初始化，不把单一姿态误当作完整的三轴加速度计零偏标定。
- ICM42688 使用 `±4 g`、`±1000 dps`、100 Hz，并将二阶 UI 低通滤波带宽显式配置为约 25 Hz。姿态解算采用六轴 AHRS，OLED 每约 100 ms 更新一次 Pitch、Roll、Yaw。
- 车体坐标为 NWU：X 指向车头、Y 指向左侧、Z 指向上方。本车模块实际为传感器 `+Y` 向车头、`+X` 向左，因此右手系 `+Z` 向下；`g_icm42688_service_config` 已配置为 `车体X←+传感器Y、车体Y←+传感器X、车体Z←-传感器Z`。
- Yaw 在校准完成时置 0，表示上电后的相对航向，范围为 `-180°~180°`。六轴器件没有磁力计，静止零偏只能减小漂移，不能提供绝对航向或消除长期温漂。
- 姿态算法基于固定提交 `015d68494274b479b5996bff2530ecbcfdc266f2` 的 x-io Fusion，MIT 许可证保存在 `ml_libs/FUSION_LICENSE.md`。

### 模块分层与调用

- `icm42688.c/.h`：负责 I²C 寄存器配置和六轴物理量读取。
- `imu_attitude.c/.h`：负责轴映射、陀螺仪零偏校准和 Fusion 姿态解算。
- `icm42688_service.c/.h`：负责 TIMG8 时基、100 Hz 非阻塞调度、校准状态和读取错误恢复。
- `main.c`：只根据服务事件刷新 OLED，不直接读取传感器或调用 Fusion。

最小调用方式：

```c
icm42688_service_t service;
icm42688_service_output_t output;

__enable_irq();
status = icm42688_service_init(&service, &config);
while (status == ML_STATUS_OK) {
    icm42688_service_event_t event =
        icm42688_service_poll(&service, &output);
    if (event == ICM42688_SERVICE_EVENT_ANGLES_UPDATED) {
        /* output.angles.pitch_deg / roll_deg / yaw_deg */
    }
}
```

## 其他网表网络

以下引脚已在 `ml_board.h` 集中定义，但模板不为其新增完整驱动：

| 网络 | 引脚 |
| --- | --- |
| IMU_SCLK / MOSI / MISO | PA17 / PB22 / PA16 |
| IMU_CS_A / CS_G / CS_P | PA25 / PA2 / PB6 |
| IMU_INT_A / INT_G / INT_T | PB1 / PB17 / PA23 |
| KEY_UP / DOWN / LEFT | PA14 / PA24 / PA15 |
| KEY_RIGHT / CENTER | PB25 / PB24 |
| BUZZER / ADC_read | PA21 / PA27 |
| AD0 / AD1 / AD2 | PB21 / PA30 / PB0 |
| C1 / C2 / C3 / C8 | PA31 / PA12 / PB8 / PB10 |

五向按键公共端接地，使用时必须配置为上拉输入，按下为低电平。

## PWM 路由与复用冲突

通用 PWM 驱动支持以下固定路由：

| 定时器通道 | 引脚 |
| --- | --- |
| TIMG0_CH0 / CH1 | PA12 / PA13 |
| TIMG6_CH0 / CH1 | PB6 / PB7 |
| TIMG7_CH0 / CH1 | PA28 / PB19 |
| TIMG8_CH0 / CH1 | PA26 / PA27 |
| TIMG12_CH0 / CH1 | PB20 / PB24 |

初始化时会登记驱动之间能够自动检测的资源冲突：

| 引脚或资源 | 冲突用途 |
| --- | --- |
| PA13 | 电机 AIN1 / TIMG0_CH1 |
| PB7 | 电机 BIN2 / TIMG6_CH1 |
| PA26 | UART3_TX / TIMG8_CH0 |
| PB22 | 核心板 LED / IMU_MOSI |
| PB24 | KEY_CENTER / TIMG12_CH1 |
| TIMG7 | CH0 电机 PWM / CH1 加热 PWM 共用频率 |

同一定时器的两个 PWM 通道必须使用相同频率；第二个通道请求不同频率时返回 `ML_STATUS_BUSY`。未提供驱动的核心板 LED、IMU SPI 和按键由应用层负责遵守冲突表。

## 驱动行为

- 初始化和传输函数使用 `ml_status_t` 返回参数错误、未初始化、超时、资源占用、I2C 无 ACK、缓冲区状态或设备未找到等状态。
- 软件 I2C 只主动拉低总线，高电平通过输入高阻释放，支持 ACK、时钟拉伸超时和 9 脉冲总线恢复。
- EXTI 同时支持 PA0-PA27 与 PB0-PB27；共享 `GROUP1_IRQHandler` 会分别处理 GPIOA 和 GPIOB 的全部待处理中断。
- UART RX 使用 64 字节环形缓冲区；满时丢弃新字节并累计溢出次数。
- 电机初始化先设置四个方向脚和两个 PWM 通道为安全零输出，接通电机电源前应先完成空载波形检查。
 - 编码器计数为 `volatile int32_t`，应用应使用 `encoder_get_and_clear()` 原子读取并清零。当前驱动在 A 相下降沿 1×计数；MG310 的 13 PPR、1:20 减速比对应 260 tick/轮圈，48 mm 名义轮径的理论值为 0.57999 mm/tick，正式运行仍以左右轮实测值为准。
 - 扩展板电源按三条电源轨使用：12 V 给 TB6612 `VM`/电机，5 V 给核心板逻辑、OLED 和 TB6612 `VCC/STBY`，3.3 V 飞线给 LF04 与编码器；不要把单一电压同时接到所有端子。

## 构建与验收

全量构建命令：

```text
F:\\xinkil\\UV4\\UV4.exe -r user\\project.uvprojx -j0
```

提交前应确认：

- 构建结果为 `0 Error(s), 0 Warning(s)`。
- 含中文的 `.c/.h` 文件为 UTF-8 BOM，注释不存在乱码或替换字符。
- 链接结果移除了未使用的 `pid_control`、`MPU6050_Init`、`MPU6050_GetData` 和 `SYSCFG_DL_init`。
- 烧录前必须完成 `ROBOT_SETUP.md` 中的左右轮、传感器至轮轴距离、有效轮距和圆周补偿标定。
- 硬件按“架空轮胎 → 低速直线 → A 点横线 → 单半径圆 → 完整任务”的顺序逐级验收。
