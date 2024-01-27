#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_usart.h"
#include "stdio.h"

#define HCSR04_TRIG_PIN    LL_GPIO_PIN_2
#define HCSR04_ECHO_PIN    LL_GPIO_PIN_1

void SystemClock_Config(void);
void Configure_GPIO(void);
void Configure_TIM2(void);
void Configure_USART1(void);
void delay_us(uint32_t us);
void HCSR04_StartTrigger(void);
void HCSR04_MeasureDistance(void);

volatile uint32_t capture_val = 0;

int main(void) {
    SystemClock_Config();
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	
    Configure_GPIO();
    Configure_TIM2();
    Configure_USART1();

    while (1) {
        HCSR04_StartTrigger();
        HCSR04_MeasureDistance();

        // Convert distance to string for USART1 transmission
        char distance_str[50];
        sprintf(distance_str, "Distance: %lu cm\n", capture_val);
        
        // Transmit distance via USART1
        for (int i = 0; distance_str[i] != '\0'; i++) {
            while (!LL_USART_IsActiveFlag_TXE(USART1)) {}
            LL_USART_TransmitData8(USART1, distance_str[i]);
        }

        // Wait before triggering again (adjust as needed)
        LL_mDelay(1000);
    }
}

void Configure_GPIO(void) {
    // Enable GPIOA clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    // Configure TRIG pin as output
    LL_GPIO_InitTypeDef gpio_initstruct = {
        .Pin = HCSR04_TRIG_PIN,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .Pull = LL_GPIO_PULL_NO
    };
    LL_GPIO_Init(GPIOA, &gpio_initstruct);

    // Configure ECHO pin as input
    gpio_initstruct.Pin = HCSR04_ECHO_PIN;
    gpio_initstruct.Mode = LL_GPIO_MODE_INPUT;
    gpio_initstruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &gpio_initstruct);
}

void Configure_TIM2(void) {
    // Enable TIM2 clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    // Configure TIM2 for input capture
    LL_TIM_InitTypeDef tim_initstruct = {
        .Prescaler = 0,
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        .Autoreload = 0xFFFFFFFF,
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV1
    };
    LL_TIM_Init(TIM2, &tim_initstruct);

    LL_TIM_IC_InitTypeDef tim_ic_initstruct = {
        .ICPolarity = LL_TIM_IC_POLARITY_BOTHEDGE,  // ??????? DIRECTTI/INDIRECTTI ???? BOTHEDGE
        .ICPrescaler = LL_TIM_ICPSC_DIV1,
        .ICFilter = 0
    };
    LL_TIM_IC_Init(TIM2, LL_TIM_CHANNEL_CH1, &tim_ic_initstruct);

    // Enable update interrupt
    LL_TIM_EnableIT_UPDATE(TIM2);

    // Enable TIM2
    LL_TIM_EnableCounter(TIM2);
}

void Configure_USART1(void) {
    // Enable USART1 clock
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    // USART1 GPIO Configuration
    // Configure TX Pin
    LL_GPIO_InitTypeDef gpio_initstruct = {
        .Pin        = LL_GPIO_PIN_9,
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_UP
    };
    LL_GPIO_Init(GPIOA, &gpio_initstruct);

    // Configure RX Pin
    gpio_initstruct.Pin = LL_GPIO_PIN_10;
    LL_GPIO_Init(GPIOA, &gpio_initstruct);

    // USART1 Configuration
    LL_USART_InitTypeDef usart_initstruct = {
        .BaudRate            = 9600,
        .DataWidth           = LL_USART_DATAWIDTH_8B,
        .StopBits            = LL_USART_STOPBITS_1,
        .Parity              = LL_USART_PARITY_NONE,
        .TransferDirection   = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE
    };
    LL_USART_Init(USART1, &usart_initstruct);

    // Enable USART1
    LL_USART_Enable(USART1);
}

void delay_us(uint32_t us) {
    uint32_t delay_count = us * (SystemCoreClock / 1000000) / 10;
    while (delay_count--) {}
}

void HCSR04_StartTrigger(void) {
    // Send a 10us pulse to trigger pin
    LL_GPIO_SetOutputPin(GPIOA, HCSR04_TRIG_PIN);
    delay_us(10);
    LL_GPIO_ResetOutputPin(GPIOA, HCSR04_TRIG_PIN);
}

void HCSR04_MeasureDistance(void) {
    // Wait for rising edge on ECHO pin
    while (!LL_TIM_IsActiveFlag_CC1(TIM2)) {}
    LL_TIM_ClearFlag_CC1(TIM2);

    // Start timer
    LL_TIM_EnableCounter(TIM2);

    // Wait for falling edge on ECHO pin
    while (!LL_TIM_IsActiveFlag_CC1(TIM2)) {}

    // Stop timer
    LL_TIM_DisableCounter(TIM2);

    // Read captured value
    capture_val = LL_TIM_IC_GetCaptureCH1(TIM2);
    capture_val = (capture_val * 10) / 58; // Convert to distance in cm
}

void TIM2_IRQHandler(void) {
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        // Clear the update interrupt flag
        LL_TIM_ClearFlag_UPDATE(TIM2);

        // Reset the captured value
        capture_val = 0;
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
