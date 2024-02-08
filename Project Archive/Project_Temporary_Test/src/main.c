#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_exti.h"

uint16_t ref_value = 100; //for change duty cycle
void TIM_OC_GPIO_Config(void);
void TIM_OC_Config(void);
void TIM_BASE_Config(void);
void PA0_EXTI_Config(void);
void GPIO_Config(void);
void SystemClock_Config(void);
void DisplayNumber(int number);

int main(void)
{
		SystemClock_Config();
		TIM_OC_Config();
		PA0_EXTI_Config();
		GPIO_Config();
		//Set Enable IC and Set PIN 5 for Reverse motor
		uint32_t digit[4] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3};

		//configure ltc4727js
		LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB); 
		LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);

		LL_GPIO_InitTypeDef ltc4727_initstruct = {
				.Mode = LL_GPIO_MODE_OUTPUT,
				.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
				.Pull = LL_GPIO_PULL_NO,
				.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
				.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15
		};
		LL_GPIO_Init(GPIOB, &ltc4727_initstruct);

		ltc4727_initstruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3; 
		LL_GPIO_Init(GPIOC, &ltc4727_initstruct);

		while (1) {
				DisplayNumber(ref_value);	
		}
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
		
		if (number >= 1000) {temporary_Seg[0] = segType[number / 1000];} // Thousands
		if (number >= 100) {temporary_Seg[1] = segType[number % 1000 / 100];} // Hundreds
		if (number >= 10) {temporary_Seg[2] = segType[(number % 100) / 10];} // Tens
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

//PA0 Setup
void GPIO_Config(void) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		//PA0
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
		LL_GPIO_InitTypeDef gpio_initstructure = {
				.Mode = LL_GPIO_MODE_INPUT,
				.Pin = LL_GPIO_PIN_0,
				.Pull = LL_GPIO_PULL_NO,
				.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH
		};
		LL_GPIO_Init(GPIOA, &gpio_initstructure);
}

//EXTI PA0 Setup
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

//EXTI for Change duty Cycle by ref_value
void EXTI0_IRQHandler(void) {
		if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) == SET) {
				//check if ref start at 100? :yes duty cycle -20 :no reset value to 100
				ref_value = ref_value>1?ref_value-20:100;
				LL_TIM_OC_SetCompareCH2(TIM3,ref_value);
		}
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}

//TIM3_BASE_Config
void TIM_BASE_Config(void) {
		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
		//Time-base configure
		LL_TIM_InitTypeDef timbase_initstructure = {
				.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2,
				.CounterMode = LL_TIM_COUNTERMODE_UP,
				.Autoreload = 100 - 1, //Set ARR to 100 for Compare
				.Prescaler = 16000 - 1
		};
		LL_TIM_Init(TIM3, &timbase_initstructure);
		LL_TIM_EnableCounter(TIM3);
}
//TIMOC_Setup to PIN5
void TIM_OC_GPIO_Config(void) {
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
		LL_GPIO_InitTypeDef gpio_initstructure = {
				.Mode = LL_GPIO_MODE_ALTERNATE,
				.Alternate = LL_GPIO_AF_2,
				.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
				.Pin = LL_GPIO_PIN_5,
				.Pull = LL_GPIO_PULL_NO,
				.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH
		};
		LL_GPIO_Init(GPIOB, &gpio_initstructure);
}
//TIM_OC_Config
void TIM_OC_Config() {
		TIM_OC_GPIO_Config();
		TIM_BASE_Config();
		LL_TIM_OC_InitTypeDef tim_oc_initstructure = {
				.OCState = LL_TIM_OCSTATE_DISABLE,
				.OCMode = LL_TIM_OCMODE_PWM1,
				.OCPolarity = LL_TIM_OCPOLARITY_HIGH,
				.CompareValue = 100
		};
		LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);
		/*Start Output Compare in PWM Mode*/
		LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
		LL_TIM_EnableCounter(TIM3);
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
