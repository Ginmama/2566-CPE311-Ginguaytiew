// Basic header-----------------------------------------
#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
// Segment header---------------------------------------
#include "stm32l1xx_ll_pwr.h"

void Configure_GPIO(void);
void DisplayNumber(int number);
void SystemClock_Config(void);

int main(void) {
    SystemClock_Config();
    Configure_GPIO();
	
		int inputNumber = 4561;

    while (1) {
        // Display the number
        DisplayNumber(inputNumber);
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
		
		if ((number / 1000) > 0) {temporary_Seg[0] = segType[number / 1000];} // Thousands
		if (((number % 1000) / 100) > 0) {temporary_Seg[1] = segType[number % 1000 / 100];} // Hundreds
		if (((number % 100) / 10) > 0) {temporary_Seg[2] = segType[(number % 100) / 10];} // Tens
		temporary_Seg[3] = segType[number % 10]; // Units

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