#include "ml_gpio.h"

/* Bit 7 selects GPIOB; bits 0-4 hold the GPIO pin number. */
static const uint8_t g_iomux_gpio_map[60] = {
    0U, 1U, 28U, 29U, 30U, 31U, 2U, 3U, 4U, 5U, 6U,
    0x80U, 0x81U, 7U, 0x82U, 0x83U, 0x84U, 0x85U,
    8U, 9U, 10U, 11U, 0x86U, 0x87U, 0x88U, 0x89U,
    0x8AU, 0x8BU, 0x8CU, 0x8DU, 0x8EU, 0x8FU, 0x90U,
    12U, 13U, 14U, 15U, 16U, 17U, 18U, 19U, 20U,
    0x91U, 0x92U, 0x93U, 21U, 22U, 0x94U, 0x95U, 0x96U,
    0x97U, 0x98U, 23U, 24U, 25U, 0x99U, 0x9AU, 0x9BU,
    26U, 27U
};

static bool gpio_port_is_valid(GPIO_Regs *gpio)
{
    return (gpio == GPIOA) || (gpio == GPIOB);
}

static bool gpio_pin_mapping_is_valid(
    GPIO_Regs *gpio, uint32_t pin, GPIOn_enum iomux)
{
    uint8_t mapped;
    GPIO_Regs *mapped_port;
    uint32_t mapped_pin;

    if ((uint32_t) iomux > (uint32_t) IOMUX_PINCM60) {
        return false;
    }
    mapped = g_iomux_gpio_map[(uint32_t) iomux];
    mapped_port = ((mapped & 0x80U) != 0U) ? GPIOB : GPIOA;
    mapped_pin = UINT32_C(1) << (mapped & 0x1FU);
    return (gpio == mapped_port) && (pin == mapped_pin);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief		GPIO初始化
// @param	  gpio	  选择GPIO端口	
// @param	  pins    选择引脚号
// @param	  gpion   选择对应引脚 
//                  注意这里有所不同 其实就是多加一个参数 比如要初始化PA1引脚 那这个参数就是PA1
// @return		void  
// Sample usage:		gpio_init(GPIOA, DL_GPIO_PIN_1, PA1, OUT);		
//-------------------------------------------------------------------------------------------------------------------
ml_status_t gpio_init(
    GPIO_Regs *gpio, uint32_t pins, GPIOn_enum gpion, GPIO_Mode_enum mode)
{
    if (!gpio_port_is_valid(gpio) || (pins == 0U) ||
        ((pins & (pins - 1U)) != 0U) ||
        !gpio_pin_mapping_is_valid(gpio, pins, gpion)) {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    if (mode == OUT) {
        DL_GPIO_initDigitalOutput((uint32_t) gpion);
        gpio_set(gpio, pins, 0U);
        DL_GPIO_enableOutput(gpio, pins);
    } else if (mode == IN_DOWN) {
        DL_GPIO_initDigitalInputFeatures((uint32_t) gpion,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    } else if (mode == IN_UP) {
        DL_GPIO_initDigitalInputFeatures((uint32_t) gpion,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    } else if (mode == IN_FLOAT) {
        DL_GPIO_initDigitalInputFeatures((uint32_t) gpion,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
            DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    } else {
        return ML_STATUS_INVALID_ARGUMENT;
    }

    return ML_STATUS_OK;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief	  设置GPIO电平
// @param	  gpio	  选择GPIO端口	
// @param	  pins    选择引脚号
// @param	  status   设置引脚电平(0/1)
// @return		void  
// Sample usage:		gpio_set(GPIOB, DL_GPIO_PIN_3, 1);
//-------------------------------------------------------------------------------------------------------------------
void gpio_set(GPIO_Regs *gpio, uint32_t pins, uint8_t status)
{
	if (!gpio_port_is_valid(gpio) || (pins == 0U)) {
		return;
	}
	if(status)
		gpio->DOUTSET31_0 = pins;
	else
		gpio->DOUTCLR31_0 = pins;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief	  翻转GPIO电平
// @param	  gpiox	  选择GPIO端口	
// @param	  pinx    选择引脚号
// Sample usage:		gpio_toggle(GPIOB, DL_GPIO_PIN_3);
//-------------------------------------------------------------------------------------------------------------------
void gpio_toggle(GPIO_Regs *gpio, uint32_t pins)
{
	if (!gpio_port_is_valid(gpio) || (pins == 0U)) {
		return;
	}
	gpio->DOUTTGL31_0 = pins;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief	  读取GPIO电平
// @param	  gpio	  选择GPIO端口	
// @param	  pins    选择引脚号
// @return	  引脚电平 
// Sample usage:		uint8_t state = gpio_get(GPIOB, DL_GPIO_PIN_3);
//-------------------------------------------------------------------------------------------------------------------
uint8_t gpio_get(GPIO_Regs *gpio, uint32_t pins)
{
	uint32_t temp;
	if (!gpio_port_is_valid(gpio) || (pins == 0U)) {
		return 0U;
	}
	temp = gpio->DIN31_0 & pins;
	if(temp > 0)
		return 1;
	else 
		return 0;
}

