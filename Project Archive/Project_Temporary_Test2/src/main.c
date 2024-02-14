#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_exti.h"

void SystemClock_Config(void);
void PD2_EXTI_Config(void);
void PA0_EXTI_Config(void);

uint16_t ref_value = 0;

int main(void) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
		SystemClock_Config();
		PA0_EXTI_Config();
		PD2_EXTI_Config();
	
		LL_GPIO_InitTypeDef GPIO_InitStruct = {
				.Mode = LL_GPIO_MODE_OUTPUT,
				.Pin = LL_GPIO_PIN_6,
				.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
				.Pull = LL_GPIO_PULL_NO,
				.Speed = LL_GPIO_SPEED_FREQ_HIGH
		};
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
		LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		while(1){
				//
		}
};

void PA0_EXTI_Config(void) {
		LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
		//PA0_EXTI Setup
		LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
		LL_EXTI_InitTypeDef PA0_EXTI_InitStruct = {
				.Line_0_31 = LL_EXTI_LINE_0,
				.LineCommand = ENABLE,
				.Mode = LL_EXTI_MODE_IT,
				.Trigger = LL_EXTI_TRIGGER_RISING
		};
		LL_EXTI_Init(&PA0_EXTI_InitStruct);
		//NVIC Setup
		NVIC_EnableIRQ((IRQn_Type)6);
		NVIC_SetPriority((IRQn_Type)6, 0);
}

void PD2_EXTI_Config(void) {
    LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    // PD2_EXTI Setup
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTD, LL_SYSCFG_EXTI_LINE2); // ????????????????? EXTI source ???? Port D ??? line 2
    LL_EXTI_InitTypeDef PD2_EXTI_InitStruct = {
        .Line_0_31 = LL_EXTI_LINE_2, // ??????? EXTI line ???? line 2
        .LineCommand = ENABLE,
        .Mode = LL_EXTI_MODE_IT,
        .Trigger = LL_EXTI_TRIGGER_RISING
    };
    LL_EXTI_Init(&PD2_EXTI_InitStruct);
    // NVIC Setup
    NVIC_EnableIRQ(EXTI2_IRQn); // ??????????? EXTI2_IRQn
    NVIC_SetPriority(EXTI2_IRQn, 0);
}

void EXTI0_IRQHandler(void) {
		if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
				if(ref_value == 0) {
						LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
						LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
						ref_value = 1;
				}
        else {
						LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
						LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
						ref_value = 0;
				}
		}
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

void EXTI2_IRQHandler(void) { // ??????????? ISR ??? EXTI3_IRQHandler ???? EXTI2_IRQHandler
    if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2)) { // ??????????? EXTI_LINE_2
        if(ref_value == 0) {
            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6);
            LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
            ref_value = 1;
        }
        else {
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
            ref_value = 0;
        }
    }
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2); // ??????????? EXTI_LINE_2
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
