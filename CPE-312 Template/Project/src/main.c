#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"

void SystemClock_Config(void);
void TIM_OC_Config(void);

LL_GPIO_InitTypeDef ltc4727_initstruct;
uint32_t seg[4] = {	/*SEG1*/LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
										/*SEG2*/LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
										/*SEG3*/LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
										/*SEG4*/LL_GPIO_PIN_2 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15};

uint32_t digit[4] = {LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3};

void Configure_GPIO(void){
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

	// Configure GPIO for segments
	LL_GPIO_InitTypeDef ltc4727_initstruct = {
		.Mode = LL_GPIO_MODE_OUTPUT,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15
	};
	LL_GPIO_Init(GPIOB, &ltc4727_initstruct);

	// Configure GPIO for digits
	ltc4727_initstruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
	LL_GPIO_Init(GPIOC, &ltc4727_initstruct);
}

int main(void){
	SystemClock_Config();
	Configure_GPIO();
	uint8_t i = 1;

	// GPIO initialization
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	LL_GPIO_InitTypeDef gpio_initstruct = {
		.Mode = LL_GPIO_MODE_OUTPUT,
		.OutputType = LL_GPIO_OUTPUT_PUSHPULL,
		.Pull = LL_GPIO_PULL_NO,
		.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
		.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_7
	};
	LL_GPIO_Init(GPIOB, &gpio_initstruct);

	// Set and reset GPIO pins
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);

	while(1){
		for (uint8_t i = 0; i < 4; ++i){
			// Turn off all digits
			LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3);

			// Turn off all segments
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 |
										   LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15);

			// Display the segment pattern for the current digit
			LL_GPIO_SetOutputPin(GPIOB, seg[i]);
			LL_GPIO_SetOutputPin(GPIOC, digit[i]);

			// Add a delay for visibility (adjust as needed)
			LL_mDelay(1);
		}
	}
}

void SystemClock_Config(void){
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0){};
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0){
		/* HSI configuration and activation */
		LL_RCC_HSI_Enable();
		while(LL_RCC_HSI_IsReady() != 1){};
  }
  
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1){};
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){};
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz							 */
  /* This frequency can be calculated through LL RCC macro						  */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}
