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
void l293d_Config(void);
void PA0_EXTI_Config(void);
void GPIO_Config(void);
void SystemClock_Config(void);


int main(void)
{
	SystemClock_Config();
	l293d_Config();
	TIM_OC_Config();
	PA0_EXTI_Config();
	GPIO_Config();
	//Set Enable IC and Set PIN 5 for Reverse motor
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
	
	LL_GPIO_InitTypeDef ltc4727_initstruct;
	uint8_t i;

         //create segment for 100-0 
       //100 dutycycle
	uint32_t seg100 [4] = {0,
											LL_GPIO_PIN_10 | LL_GPIO_PIN_11, 
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 };
         //80 dutycycle
	uint32_t seg80 [4] = {0, 0, 
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 };
	//60 dutycycle
	uint32_t seg60 [4] = {0, 0, 
										  LL_GPIO_PIN_2 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 };
	//40 dutycycle
	uint32_t seg40 [4] = {0, 0,
											LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 };
	//20 dutycycle
	uint32_t seg20 [4] = {0, 0,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10  |LL_GPIO_PIN_12 | LL_GPIO_PIN_13  | LL_GPIO_PIN_15,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 };
	
	uint32_t seg0 [4] = {0, 0, 0,
											LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 |LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14};											
	uint32_t digit [4] ={LL_GPIO_PIN_0, LL_GPIO_PIN_1, LL_GPIO_PIN_2, LL_GPIO_PIN_3};
	
	//configure ltc4727js
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB); 
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC);
	
	ltc4727_initstruct.Mode = LL_GPIO_MODE_OUTPUT;
	ltc4727_initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	ltc4727_initstruct.Pull = LL_GPIO_PULL_NO;
	ltc4727_initstruct.Speed =LL_GPIO_SPEED_FREQ_VERY_HIGH;
	ltc4727_initstruct. Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 |
	LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	LL_GPIO_Init(GPIOB, &ltc4727_initstruct);

	ltc4727_initstruct. Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3; 
	LL_GPIO_Init(GPIOC, &ltc4727_initstruct);
	
	
	while (1)
	{
		
		for (i= 0; i < 4; ++i)
	{
			LL_GPIO_ResetOutputPin (GPIOC, LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3); //Write 0 to GPIOC port LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL GPIO_PIN_11 | LL_GPIO_PIN_12 |
			LL_GPIO_ResetOutputPin (GPIOB, LL_GPIO_PIN_2 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 |
   		LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15); //Reset all segment (PB2, PB10-PB15)
                    //show number on segment 
		  switch(ref_value) {
        case 100:
					LL_GPIO_SetOutputPin (GPIOB, seg100[i]);
					LL_GPIO_SetOutputPin (GPIOC, digit[i]);
					LL_mDelay(0); //USE for DEBUG increase delay to see what's happenning when 7-seg is lit
					break;
				case 80:
					LL_GPIO_SetOutputPin (GPIOB, seg80[i]);
					LL_GPIO_SetOutputPin (GPIOC, digit[i]);
					LL_mDelay(0); //USE for DEBUG increase delay to see what's happenning when 7-seg is lit
					break;
				case 60:
					LL_GPIO_SetOutputPin (GPIOB, seg60[i]);
					LL_GPIO_SetOutputPin (GPIOC, digit[i]);
					LL_mDelay(0); //USE for DEBUG increase delay to see what's happenning when 7-seg is lit
					break;
				case 40:
					LL_GPIO_SetOutputPin (GPIOB, seg40[i]);
					LL_GPIO_SetOutputPin (GPIOC, digit[i]);
					LL_mDelay(0); //USE for DEBUG increase delay to see what's happenning when 7-seg is lit
					break;
				case 20:
					LL_GPIO_SetOutputPin (GPIOB, seg20[i]);
					LL_GPIO_SetOutputPin (GPIOC, digit[i]);
					LL_mDelay(0); //USE for DEBUG increase delay to see what's happenning when 7-seg is lit
					break;
				default:
					LL_GPIO_SetOutputPin (GPIOB, seg0[i]);
					LL_GPIO_SetOutputPin (GPIOC, digit[i]);
					LL_mDelay(0); //USE for DEBUG increase delay to see what's happenning when 7-seg is lit
		}
	}		
 }
}

//PA0 Setup
void GPIO_Config(void)
{
 LL_GPIO_InitTypeDef gpio_initstructure;
 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
 //PA0
 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
 gpio_initstructure.Mode = LL_GPIO_MODE_INPUT;
 gpio_initstructure.Pin = LL_GPIO_PIN_0;
 gpio_initstructure.Pull = LL_GPIO_PULL_NO;
 gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
 LL_GPIO_Init(GPIOA, &gpio_initstructure);
}

//EXTI PA0 Setup
void PA0_EXTI_Config(void)
{
 LL_EXTI_InitTypeDef PA0_EXTI_InitStruct;
 LL_APB1_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
 //PA0_EXTI Setup
 LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
 PA0_EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
 PA0_EXTI_InitStruct.LineCommand = ENABLE;
 PA0_EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
 PA0_EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
 LL_EXTI_Init(&PA0_EXTI_InitStruct);
 //NVIC Setup
 NVIC_EnableIRQ((IRQn_Type)6);
 NVIC_SetPriority((IRQn_Type)6,0);
}

//EXTI for Change duty Cycle by ref_value
void EXTI0_IRQHandler(void)
{
 if(LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) == SET)
 {
	 //check if ref start at 100? :yes duty cycle -20 :no reset value to 100
	 ref_value = ref_value>1?ref_value-20:100;
	 LL_TIM_OC_SetCompareCH2(TIM3,ref_value);
 }
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}
//IC l293d Config
void l293d_Config(void)
{
  LL_GPIO_InitTypeDef l293d_init;
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  l293d_init.Mode = LL_GPIO_MODE_OUTPUT;
  l293d_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  l293d_init.Pull = LL_GPIO_PULL_NO;
  l293d_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  l293d_init.Pin = LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOB, &l293d_init);
}
//TIM3_BASE_Config
void TIM_BASE_Config(void)
{
  LL_TIM_InitTypeDef timbase_initstructure;
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  //Time-base configure
  timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
  timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
  timbase_initstructure.Autoreload = 100 - 1; //Set ARR to 100 for Compare
  timbase_initstructure.Prescaler = 16000 - 1;
  LL_TIM_Init(TIM3, &timbase_initstructure);
  LL_TIM_EnableCounter(TIM3);
}
//TIMOC_Setup to PIN5
void TIM_OC_GPIO_Config(void)
{
 LL_GPIO_InitTypeDef gpio_initstructure;
 LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
 gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
 gpio_initstructure.Alternate = LL_GPIO_AF_2;
 gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
 gpio_initstructure.Pin = LL_GPIO_PIN_5;
 gpio_initstructure.Pull = LL_GPIO_PULL_NO;
 gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
 LL_GPIO_Init(GPIOB, &gpio_initstructure);
}
//TIM_OC_Config
void TIM_OC_Config()
{
	LL_TIM_OC_InitTypeDef tim_oc_initstructure;
	TIM_OC_GPIO_Config();
	TIM_BASE_Config();
	tim_oc_initstructure.OCState = LL_TIM_OCSTATE_DISABLE;
	tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
	tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	tim_oc_initstructure.CompareValue = 100;
	LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &tim_oc_initstructure);
	/*Start Output Compare in PWM Mode*/
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM3);
}

void SystemClock_Config(void)
{
  /* Enable ACC64 access and set FLASH latency */ 
  LL_FLASH_Enable64bitAccess();; 
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
	
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);

  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  /* Set systick to 1ms in using frequency set to 32MHz                             */
  /* This frequency can be calculated through LL RCC macro                          */
  /* ex: __LL_RCC_CALC_PLLCLK_FREQ (HSI_VALUE, LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3); */
  LL_Init1msTick(32000000);
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(32000000);
}
