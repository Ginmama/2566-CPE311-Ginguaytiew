#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_exti.h"
#include "stm32l1xx_ll_i2c.h"

void TIM_OC_Config(void);
void TIM_BASE_Config(void);
void GPIO_Config(void);
void SystemClock_Config(void);
void DisplayNumber(int number);
#define VL53L0X_ADDRESS 0x29
void VL53L0X_Init(void);
void Configure_GPIO(void);
void VL53L0X_ReadDistance(uint16_t *distance);
uint16_t laser = 0; //for adjust value

int main(void)
{   
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); 
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    
    SystemClock_Config();
    TIM_OC_Config();
    GPIO_Config();
    VL53L0X_Init();
	  DisplayNumber(0);
		Configure_GPIO();

   
    while (1) {
        uint16_t distance;
        VL53L0X_ReadDistance(&distance);
        laser = distance; // Assign the distance value to laser variable
        DisplayNumber(laser);
    }
}

void VL53L0X_Init(void)
{
    LL_I2C_InitTypeDef i2c_init_struct = {0};
    i2c_init_struct.PeripheralMode = LL_I2C_MODE_I2C;
    i2c_init_struct.ClockSpeed = 100000; // I2C clock speed (100 kHz)
    i2c_init_struct.DutyCycle = LL_I2C_DUTYCYCLE_2;
    i2c_init_struct.OwnAddress1 = 0;
    i2c_init_struct.TypeAcknowledge = LL_I2C_ACK;
    LL_I2C_Init(I2C1, &i2c_init_struct);
    LL_I2C_Enable(I2C1);
}

void VL53L0X_ReadDistance(uint16_t *distance)
{
    // Start I2C communication
    LL_I2C_GenerateStartCondition(I2C1);
    while (!LL_I2C_IsActiveFlag_SB(I2C1))
    {
        // Wait for start condition
    }

    // Send device address
    LL_I2C_TransmitData8(I2C1, VL53L0X_ADDRESS);
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1))
    {
        // Wait for address sent
    }

    // Clear ADDR flag
    LL_I2C_ClearFlag_ADDR(I2C1);

    // Read data
    LL_I2C_TransmitData8(I2C1, 0x00); // Send register address to read from
    while (!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
        // Wait for TXE flag
    }

    LL_I2C_GenerateStartCondition(I2C1); // Restart condition
    while (!LL_I2C_IsActiveFlag_SB(I2C1))
    {
        // Wait for start condition
    }

    LL_I2C_TransmitData8(I2C1, VL53L0X_ADDRESS | 0x01); // Send device address with read bit
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1))
    {
        // Wait for address sent
    }

    // Clear ADDR flag
    LL_I2C_ClearFlag_ADDR(I2C1);

    // Receive data
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
        // Wait for RXNE flag
    }
    uint8_t dataLSB = LL_I2C_ReceiveData8(I2C1);
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
        // Wait for RXNE flag
    }
    uint8_t dataMSB = LL_I2C_ReceiveData8(I2C1);

    // Combine data to get distance
    *distance = (dataMSB << 8) | dataLSB;
}

void TIM_OC_Config(void) {
    TIM_BASE_Config();
    LL_TIM_OC_InitTypeDef tim_oc_initstructure = {
        .OCState = LL_TIM_OCSTATE_DISABLE,
        .OCMode = LL_TIM_OCMODE_PWM1,
        .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
        .CompareValue = 100
    };
    LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM3);
}

void TIM_BASE_Config(void) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_InitTypeDef timbase_initstructure = {
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV2,
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        .Autoreload = 100 - 1, 
        .Prescaler = 16000 - 1
    };
    LL_TIM_Init(TIM3, &timbase_initstructure);
    LL_TIM_EnableCounter(TIM3);
}

void GPIO_Config(void) {
    // Configure PA3 for SDA (I2C1)
    LL_GPIO_InitTypeDef gpio_initstructure = {
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Pin = LL_GPIO_PIN_3,
        .Pull = LL_GPIO_PULL_NO,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .Alternate = LL_GPIO_AF_4 // Choose I2C1 SDA alternate function
    };
    LL_GPIO_Init(GPIOA, &gpio_initstructure);

    // Configure PB8 for SCL (I2C1)
    gpio_initstructure.Pin = LL_GPIO_PIN_8;
    LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void DisplayNumber(int number) {
    uint32_t segType[10] = {
        // Store segment pattern 0-9
        /*0*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
        /*1*/ LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
        /*2*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_15,
        /*3*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_15,
        /*4*/ LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
        /*5*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
        /*6*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
        /*7*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
        /*8*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
        /*9*/ LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15
    };

    uint32_t temporary_Seg[4] = {0}; // Store all input number
    uint32_t digit_Control[4] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3}; // Store all digit pins
    
    // Thousands
    if (number >= 1000) {
        temporary_Seg[0] = segType[number / 1000];
        number %= 1000;
    }

    // Hundreds
    if (number >= 100) {
        temporary_Seg[1] = segType[number / 100];
        number %= 100;
    }

    // Tens
    if (number >= 10) {
        temporary_Seg[2] = segType[number / 10];
        number %= 10;
    }

    // Units
    temporary_Seg[3] = segType[number];

    for (uint8_t i = 0; i < 4; ++i) {
        // Turn off all digits and segments
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 |
                                             LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);

        // Display the segment pattern for the current digit
        LL_GPIO_SetOutputPin(GPIOB, temporary_Seg[i]);
        LL_GPIO_SetOutputPin(GPIOC, digit_Control[i]);

        // Add a delay for visibility (adjust as needed)
        LL_mDelay(2);
    }
}

void Configure_GPIO(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_GPIO_InitTypeDef ltc4727_initstruct = {
        .Mode = LL_GPIO_MODE_OUTPUT,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull = LL_GPIO_PULL_NO,
        .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH
    };

    // Configure GPIO for segments
    ltc4727_initstruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
    LL_GPIO_Init(GPIOB, &ltc4727_initstruct);

    // Configure GPIO for digit
    ltc4727_initstruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOC, &ltc4727_initstruct);
}
void SystemClock_Config(void) {
    /* Enable ACC64 access and set FLASH latency */
    LL_FLASH_Enable64bitAccess();
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    /* Set Voltage scale1 as MCU will run at 32MHz */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    /* Poll VOSF bit in PWR_CSR. Wait until it is reset to 0 */
    while (LL_PWR_IsActiveFlag_VOSF() != 0) {}

    /* Enable HSI if not already activated */
    if (LL_RCC_HSI_IsReady() == 0) {
        /* HSI configuration and activation */
        LL_RCC_HSI_Enable();
        while (LL_RCC_HSI_IsReady() != 1) {}
    }

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1) {}

    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

    /* Set APB1 & APB2 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    /* Set systick to 1ms using frequency set to 32MHz */
    /* This frequency can be calculated through LL RCC macro */
    /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
    LL_Init1msTick(32000000);

    /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(32000000);
}
