#ifndef ML_BOARD_H
#define ML_BOARD_H

#ifndef CONFIG_MSPM0G350X
#define CONFIG_MSPM0G350X
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "ml_common.h"

#define ML_CPU_CLOCK_HZ                 (80000000UL)
#define ML_MFCLK_HZ                     (4000000UL)
#define ML_NVIC_PRIORITY_MAX            (3U)

/* Baseboard LEDs: anode to 3.3 V through a resistor, active-low cathode. */
#define ML_BOARD_LED1_PORT              GPIOB
#define ML_BOARD_LED1_PIN               DL_GPIO_PIN_14
#define ML_BOARD_LED1_IOMUX             IOMUX_PINCM31
#define ML_BOARD_LED1_ACTIVE_LEVEL      (0U)
#define ML_BOARD_LED2_PORT              GPIOB
#define ML_BOARD_LED2_PIN               DL_GPIO_PIN_18
#define ML_BOARD_LED2_IOMUX             IOMUX_PINCM44
#define ML_BOARD_LED2_ACTIVE_LEVEL      (0U)
#define ML_BOARD_LED3_PORT              GPIOA
#define ML_BOARD_LED3_PIN               DL_GPIO_PIN_22
#define ML_BOARD_LED3_IOMUX             IOMUX_PINCM47
#define ML_BOARD_LED3_ACTIVE_LEVEL      (0U)

/* The core-board LED conflicts with IMU_MOSI and is not the default LED. */
#define ML_CORE_BOARD_LED_PORT          GPIOB
#define ML_CORE_BOARD_LED_PIN           DL_GPIO_PIN_22
#define ML_CORE_BOARD_LED_IOMUX         IOMUX_PINCM50
#define ML_CORE_BOARD_LED_ACTIVE_LEVEL  (0U)

#define ML_BOARD_LED_PORT               ML_BOARD_LED1_PORT
#define ML_BOARD_LED_PIN                ML_BOARD_LED1_PIN
#define ML_BOARD_LED_IOMUX              ML_BOARD_LED1_IOMUX
#define ML_BOARD_LED_ACTIVE_LEVEL       ML_BOARD_LED1_ACTIVE_LEVEL

#define ML_OLED_PORT                    GPIOB
#define ML_OLED_SCL_PIN                 DL_GPIO_PIN_2
#define ML_OLED_SCL_IOMUX               IOMUX_PINCM15
#define ML_OLED_SDA_PIN                 DL_GPIO_PIN_3
#define ML_OLED_SDA_IOMUX               IOMUX_PINCM16

#define ML_MPU6050_I2C_PORT             GPIOA
#define ML_MPU6050_I2C_SCL_PIN          DL_GPIO_PIN_1
#define ML_MPU6050_I2C_SCL_IOMUX        IOMUX_PINCM2
#define ML_MPU6050_I2C_SDA_PIN          DL_GPIO_PIN_0
#define ML_MPU6050_I2C_SDA_IOMUX        IOMUX_PINCM1

/* ICM42688 I2C reuses the original IMU_SCL/IMU_SDA nets. */
#define ML_ICM42688_I2C_PORT             GPIOA
#define ML_ICM42688_I2C_SCL_PIN          DL_GPIO_PIN_1
#define ML_ICM42688_I2C_SCL_IOMUX        IOMUX_PINCM2
#define ML_ICM42688_I2C_SDA_PIN          DL_GPIO_PIN_0
#define ML_ICM42688_I2C_SDA_IOMUX        IOMUX_PINCM1

#define ML_MOTOR_A_PWM_TIMER            TIMG7
#define ML_MOTOR_A_PWM_CHANNEL          DL_TIMER_CC_0_INDEX
#define ML_MOTOR_A_PWM_PORT             GPIOA
#define ML_MOTOR_A_PWM_PIN              DL_GPIO_PIN_28
#define ML_MOTOR_A_PWM_IOMUX            IOMUX_PINCM3
#define ML_MOTOR_A_PWM_FUNCTION         IOMUX_PINCM3_PF_TIMG7_CCP0
#define ML_MOTOR_A_IN1_PORT             GPIOA
#define ML_MOTOR_A_IN1_PIN              DL_GPIO_PIN_13
#define ML_MOTOR_A_IN1_IOMUX            IOMUX_PINCM35
#define ML_MOTOR_A_IN2_PORT             GPIOB
#define ML_MOTOR_A_IN2_PIN              DL_GPIO_PIN_26
#define ML_MOTOR_A_IN2_IOMUX            IOMUX_PINCM57

#define ML_MOTOR_B_PWM_TIMER            TIMG12
#define ML_MOTOR_B_PWM_CHANNEL          DL_TIMER_CC_0_INDEX
#define ML_MOTOR_B_PWM_PORT             GPIOB
#define ML_MOTOR_B_PWM_PIN              DL_GPIO_PIN_20
#define ML_MOTOR_B_PWM_IOMUX            IOMUX_PINCM48
#define ML_MOTOR_B_PWM_FUNCTION         IOMUX_PINCM48_PF_TIMG12_CCP0
#define ML_MOTOR_B_IN1_PORT             GPIOB
#define ML_MOTOR_B_IN1_PIN              DL_GPIO_PIN_9
#define ML_MOTOR_B_IN1_IOMUX            IOMUX_PINCM26
#define ML_MOTOR_B_IN2_PORT             GPIOB
#define ML_MOTOR_B_IN2_PIN              DL_GPIO_PIN_7
#define ML_MOTOR_B_IN2_IOMUX            IOMUX_PINCM24

#define ML_ENCODER_A_PHASE_A_PORT        GPIOB
#define ML_ENCODER_A_PHASE_A_PIN         DL_GPIO_PIN_23
#define ML_ENCODER_A_PHASE_A_IOMUX       IOMUX_PINCM51
#define ML_ENCODER_A_PHASE_B_PORT        GPIOB
#define ML_ENCODER_A_PHASE_B_PIN         DL_GPIO_PIN_12
#define ML_ENCODER_A_PHASE_B_IOMUX       IOMUX_PINCM29
#define ML_ENCODER_B_PHASE_A_PORT        GPIOB
#define ML_ENCODER_B_PHASE_A_PIN         DL_GPIO_PIN_4
#define ML_ENCODER_B_PHASE_A_IOMUX       IOMUX_PINCM17
#define ML_ENCODER_B_PHASE_B_PORT        GPIOB
#define ML_ENCODER_B_PHASE_B_PIN         DL_GPIO_PIN_5
#define ML_ENCODER_B_PHASE_B_IOMUX       IOMUX_PINCM18

/* IMU SPI and interrupt nets are registered for applications to use. */
#define ML_IMU_SPI_SCLK_PORT             GPIOA
#define ML_IMU_SPI_SCLK_PIN              DL_GPIO_PIN_17
#define ML_IMU_SPI_SCLK_IOMUX            IOMUX_PINCM39
#define ML_IMU_SPI_MOSI_PORT             GPIOB
#define ML_IMU_SPI_MOSI_PIN              DL_GPIO_PIN_22
#define ML_IMU_SPI_MOSI_IOMUX            IOMUX_PINCM50
#define ML_IMU_SPI_MISO_PORT             GPIOA
#define ML_IMU_SPI_MISO_PIN              DL_GPIO_PIN_16
#define ML_IMU_SPI_MISO_IOMUX            IOMUX_PINCM38
#define ML_IMU_CS_A_PORT                 GPIOA
#define ML_IMU_CS_A_PIN                  DL_GPIO_PIN_25
#define ML_IMU_CS_A_IOMUX                IOMUX_PINCM55
#define ML_IMU_CS_G_PORT                 GPIOA
#define ML_IMU_CS_G_PIN                  DL_GPIO_PIN_2
#define ML_IMU_CS_G_IOMUX                IOMUX_PINCM7
#define ML_IMU_CS_P_PORT                 GPIOB
#define ML_IMU_CS_P_PIN                  DL_GPIO_PIN_6
#define ML_IMU_CS_P_IOMUX                IOMUX_PINCM23
#define ML_IMU_INT_A_PORT                GPIOB
#define ML_IMU_INT_A_PIN                 DL_GPIO_PIN_1
#define ML_IMU_INT_A_IOMUX               IOMUX_PINCM13
#define ML_IMU_INT_G_PORT                GPIOB
#define ML_IMU_INT_G_PIN                 DL_GPIO_PIN_17
#define ML_IMU_INT_G_IOMUX               IOMUX_PINCM43
#define ML_IMU_INT_T_PORT                GPIOA
#define ML_IMU_INT_T_PIN                 DL_GPIO_PIN_23
#define ML_IMU_INT_T_IOMUX               IOMUX_PINCM53

/* Keys connect to GND and must be configured as pull-up, active-low inputs. */
#define ML_KEY_ACTIVE_LEVEL              (0U)
#define ML_KEY_UP_PORT                   GPIOA
#define ML_KEY_UP_PIN                    DL_GPIO_PIN_14
#define ML_KEY_UP_IOMUX                  IOMUX_PINCM36
#define ML_KEY_DOWN_PORT                 GPIOA
#define ML_KEY_DOWN_PIN                  DL_GPIO_PIN_24
#define ML_KEY_DOWN_IOMUX                IOMUX_PINCM54
#define ML_KEY_LEFT_PORT                 GPIOA
#define ML_KEY_LEFT_PIN                  DL_GPIO_PIN_15
#define ML_KEY_LEFT_IOMUX                IOMUX_PINCM37
#define ML_KEY_RIGHT_PORT                GPIOB
#define ML_KEY_RIGHT_PIN                 DL_GPIO_PIN_25
#define ML_KEY_RIGHT_IOMUX               IOMUX_PINCM56
#define ML_KEY_CENTER_PORT               GPIOB
#define ML_KEY_CENTER_PIN                DL_GPIO_PIN_24
#define ML_KEY_CENTER_IOMUX              IOMUX_PINCM52

#define ML_BUZZER_PORT                   GPIOA
#define ML_BUZZER_PIN                    DL_GPIO_PIN_21
#define ML_BUZZER_IOMUX                  IOMUX_PINCM46
#define ML_HEAT_PWM_PORT                 GPIOB
#define ML_HEAT_PWM_PIN                  DL_GPIO_PIN_19
#define ML_HEAT_PWM_IOMUX                IOMUX_PINCM45
#define ML_HEAT_PWM_FUNCTION             IOMUX_PINCM45_PF_TIMG7_CCP1
#define ML_HEAT_PWM_TIMER                TIMG7
#define ML_HEAT_PWM_CHANNEL              DL_TIMER_CC_1_INDEX
#define ML_ADC_READ_PORT                 GPIOA
#define ML_ADC_READ_PIN                  DL_GPIO_PIN_27
#define ML_ADC_READ_IOMUX                IOMUX_PINCM60
#define ML_AD0_PORT                      GPIOB
#define ML_AD0_PIN                       DL_GPIO_PIN_21
#define ML_AD0_IOMUX                     IOMUX_PINCM49
#define ML_AD1_PORT                      GPIOA
#define ML_AD1_PIN                       DL_GPIO_PIN_30
#define ML_AD1_IOMUX                     IOMUX_PINCM5
#define ML_AD2_PORT                      GPIOB
#define ML_AD2_PIN                       DL_GPIO_PIN_0
#define ML_AD2_IOMUX                     IOMUX_PINCM12
#define ML_C1_PORT                       GPIOA
#define ML_C1_PIN                        DL_GPIO_PIN_31
#define ML_C1_IOMUX                      IOMUX_PINCM6
#define ML_C2_PORT                       GPIOA
#define ML_C2_PIN                        DL_GPIO_PIN_12
#define ML_C2_IOMUX                      IOMUX_PINCM34
#define ML_C3_PORT                       GPIOB
#define ML_C3_PIN                        DL_GPIO_PIN_8
#define ML_C3_IOMUX                      IOMUX_PINCM25
#define ML_C8_PORT                       GPIOB
#define ML_C8_PIN                        DL_GPIO_PIN_10
#define ML_C8_IOMUX                      IOMUX_PINCM27

#define ML_MOTOR_PWM_FREQUENCY_HZ        (1000U)
#define ML_PWM_DUTY_MAX                  (50000UL)
#define ML_MOTOR_DUTY_LIMIT              (17500UL) /* 35% safety cap */
#define ML_PWM_TIMG0_CH0_PORT            GPIOA
#define ML_PWM_TIMG0_CH0_PIN             DL_GPIO_PIN_12
#define ML_PWM_TIMG0_CH0_IOMUX           IOMUX_PINCM34
#define ML_PWM_TIMG0_CH0_FUNCTION        IOMUX_PINCM34_PF_TIMG0_CCP0
#define ML_PWM_TIMG0_CH1_PORT            GPIOA
#define ML_PWM_TIMG0_CH1_PIN             DL_GPIO_PIN_13
#define ML_PWM_TIMG0_CH1_IOMUX           IOMUX_PINCM35
#define ML_PWM_TIMG0_CH1_FUNCTION        IOMUX_PINCM35_PF_TIMG0_CCP1
#define ML_PWM_TIMG6_CH0_PORT            GPIOB
#define ML_PWM_TIMG6_CH0_PIN             DL_GPIO_PIN_6
#define ML_PWM_TIMG6_CH0_IOMUX           IOMUX_PINCM23
#define ML_PWM_TIMG6_CH0_FUNCTION        IOMUX_PINCM23_PF_TIMG6_CCP0
#define ML_PWM_TIMG6_CH1_PORT            GPIOB
#define ML_PWM_TIMG6_CH1_PIN             DL_GPIO_PIN_7
#define ML_PWM_TIMG6_CH1_IOMUX           IOMUX_PINCM24
#define ML_PWM_TIMG6_CH1_FUNCTION        IOMUX_PINCM24_PF_TIMG6_CCP1
#define ML_PWM_TIMG7_CH0_PORT            ML_MOTOR_A_PWM_PORT
#define ML_PWM_TIMG7_CH0_PIN             ML_MOTOR_A_PWM_PIN
#define ML_PWM_TIMG7_CH0_IOMUX           ML_MOTOR_A_PWM_IOMUX
#define ML_PWM_TIMG7_CH0_FUNCTION        ML_MOTOR_A_PWM_FUNCTION
#define ML_PWM_TIMG7_CH1_PORT            ML_HEAT_PWM_PORT
#define ML_PWM_TIMG7_CH1_PIN             ML_HEAT_PWM_PIN
#define ML_PWM_TIMG7_CH1_IOMUX           ML_HEAT_PWM_IOMUX
#define ML_PWM_TIMG7_CH1_FUNCTION        ML_HEAT_PWM_FUNCTION
#define ML_PWM_TIMG8_CH0_PORT            GPIOA
#define ML_PWM_TIMG8_CH0_PIN             DL_GPIO_PIN_26
#define ML_PWM_TIMG8_CH0_IOMUX           IOMUX_PINCM59
#define ML_PWM_TIMG8_CH0_FUNCTION        IOMUX_PINCM59_PF_TIMG8_CCP0
#define ML_PWM_TIMG8_CH1_PORT            GPIOA
#define ML_PWM_TIMG8_CH1_PIN             DL_GPIO_PIN_27
#define ML_PWM_TIMG8_CH1_IOMUX           IOMUX_PINCM60
#define ML_PWM_TIMG8_CH1_FUNCTION        IOMUX_PINCM60_PF_TIMG8_CCP1
#define ML_PWM_TIMG12_CH0_PORT           ML_MOTOR_B_PWM_PORT
#define ML_PWM_TIMG12_CH0_PIN            ML_MOTOR_B_PWM_PIN
#define ML_PWM_TIMG12_CH0_IOMUX          ML_MOTOR_B_PWM_IOMUX
#define ML_PWM_TIMG12_CH0_FUNCTION       ML_MOTOR_B_PWM_FUNCTION
#define ML_PWM_TIMG12_CH1_PORT           GPIOB
#define ML_PWM_TIMG12_CH1_PIN            DL_GPIO_PIN_24
#define ML_PWM_TIMG12_CH1_IOMUX          IOMUX_PINCM52
#define ML_PWM_TIMG12_CH1_FUNCTION       IOMUX_PINCM52_PF_TIMG12_CCP1

#define ML_SOFT_I2C_HALF_PERIOD_US       (2U)
#define ML_SOFT_I2C_TIMEOUT_US           (1000U)
#define ML_UART_RX_BUFFER_SIZE           (64U)
#define ML_UART_TX_TIMEOUT_LOOPS         (ML_CPU_CLOCK_HZ / 10U)
#define ML_PRINTF_UART                   UART0

#define ML_UART0_TX_PORT                 GPIOA
#define ML_UART0_TX_PIN                  DL_GPIO_PIN_10
#define ML_UART0_TX_IOMUX                IOMUX_PINCM21
#define ML_UART0_TX_FUNCTION             IOMUX_PINCM21_PF_UART0_TX
#define ML_UART0_RX_PORT                 GPIOA
#define ML_UART0_RX_PIN                  DL_GPIO_PIN_11
#define ML_UART0_RX_IOMUX                IOMUX_PINCM22
#define ML_UART0_RX_FUNCTION             IOMUX_PINCM22_PF_UART0_RX
#define ML_UART1_TX_PORT                 GPIOA
#define ML_UART1_TX_PIN                  DL_GPIO_PIN_8
#define ML_UART1_TX_IOMUX                IOMUX_PINCM19
#define ML_UART1_TX_FUNCTION             IOMUX_PINCM19_PF_UART1_TX
#define ML_UART1_RX_PORT                 GPIOA
#define ML_UART1_RX_PIN                  DL_GPIO_PIN_9
#define ML_UART1_RX_IOMUX                IOMUX_PINCM20
#define ML_UART1_RX_FUNCTION             IOMUX_PINCM20_PF_UART1_RX
#define ML_UART2_TX_PORT                 GPIOB
#define ML_UART2_TX_PIN                  DL_GPIO_PIN_15
#define ML_UART2_TX_IOMUX                IOMUX_PINCM32
#define ML_UART2_TX_FUNCTION             IOMUX_PINCM32_PF_UART2_TX
#define ML_UART2_RX_PORT                 GPIOB
#define ML_UART2_RX_PIN                  DL_GPIO_PIN_16
#define ML_UART2_RX_IOMUX                IOMUX_PINCM33
#define ML_UART2_RX_FUNCTION             IOMUX_PINCM33_PF_UART2_RX
#define ML_UART3_TX_PORT                 GPIOA
#define ML_UART3_TX_PIN                  DL_GPIO_PIN_26
#define ML_UART3_TX_IOMUX                IOMUX_PINCM59
#define ML_UART3_TX_FUNCTION             IOMUX_PINCM59_PF_UART3_TX
#define ML_UART3_RX_PORT                 GPIOB
#define ML_UART3_RX_PIN                  DL_GPIO_PIN_13
#define ML_UART3_RX_IOMUX                IOMUX_PINCM30
#define ML_UART3_RX_FUNCTION             IOMUX_PINCM30_PF_UART3_RX

typedef enum {
    ML_BOARD_RESOURCE_PA13 = 0,
    ML_BOARD_RESOURCE_PB7,
    ML_BOARD_RESOURCE_PA26,
    ML_BOARD_RESOURCE_PB22,
    ML_BOARD_RESOURCE_PB24,
    ML_BOARD_RESOURCE_COUNT
} ml_board_resource_t;

/* Source compatibility for code written against the previous template. */
#define ML_BOARD_RESOURCE_PA12_PA13 ML_BOARD_RESOURCE_PA13
#define ML_BOARD_RESOURCE_PB6_PB7   ML_BOARD_RESOURCE_PB7

typedef enum {
    ML_BOARD_OWNER_NONE = 0,
    ML_BOARD_OWNER_PWM_TIMG0,
    ML_BOARD_OWNER_PWM_TIMG6,
    ML_BOARD_OWNER_PWM_TIMG8,
    ML_BOARD_OWNER_PWM_TIMG12,
    ML_BOARD_OWNER_MOTOR,
    ML_BOARD_OWNER_ENCODER,
    ML_BOARD_OWNER_MPU6050,
    ML_BOARD_OWNER_UART3,
    ML_BOARD_OWNER_CORE_LED,
    ML_BOARD_OWNER_IMU_SPI,
    ML_BOARD_OWNER_KEY
} ml_board_owner_t;

ml_status_t board_led_init(void);
void board_led_on(void);
void board_led_off(void);
void board_led_toggle(void);
ml_status_t board_resource_claim(
    ml_board_resource_t resource, ml_board_owner_t owner);
void board_resource_release(
    ml_board_resource_t resource, ml_board_owner_t owner);

#endif
