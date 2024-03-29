#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_exti.h"

#define E_O6 (uint16_t)1318
#define F_O6 (uint16_t)1396.9
#define TIMx_PSC 3

#define ARR_CALCULATE(N) (SystemCoreClock) / ((TIMx_PSC) * (N))

void SystemClock_Config(void);
void TIM_OC_GPIO_Config(void);
void TIM_OC_Config(uint16_t);

uint8_t flag = 0;

int main(void) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
		SystemClock_Config();
    TIM_OC_Config(ARR_CALCULATE(F_O6));

    while(1) {
        if (flag == 0) {
            TIM_OC_Config(ARR_CALCULATE(E_O6)); // Play note E for 3 seconds
        } else if (flag == 1) {
            TIM_OC_Config(ARR_CALCULATE(F_O6)); // Play note F for 3 seconds
        }
        flag = (flag + 1) % 2;
        LL_mDelay(3000); // Delay for 3 seconds before changing the note
    }
}

void TIM_OC_GPIO_Config(void) {
    LL_GPIO_InitTypeDef gpio_initstructure;
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    gpio_initstructure.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_initstructure.Alternate = LL_GPIO_AF_2;
    gpio_initstructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_initstructure.Pin = LL_GPIO_PIN_9; // PB9
    gpio_initstructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;

    LL_GPIO_Init(GPIOB, &gpio_initstructure);
}

void TIM_OC_Config(uint16_t note) {
    LL_TIM_InitTypeDef timbase_initstructure;
    LL_TIM_OC_InitTypeDef tim_oc_initstructure;

    timbase_initstructure.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    timbase_initstructure.CounterMode = LL_TIM_COUNTERMODE_UP;
    timbase_initstructure.Autoreload = note - 1;
    timbase_initstructure.Prescaler = TIMx_PSC- 1;

    LL_TIM_Init(TIM4, &timbase_initstructure);
    LL_TIM_EnableCounter(TIM4);

    TIM_OC_GPIO_Config();

    tim_oc_initstructure.OCMode = LL_TIM_OCMODE_PWM1;
    tim_oc_initstructure.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    tim_oc_initstructure.CompareValue = note / 2;

    LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &tim_oc_initstructure);

    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM4);
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
