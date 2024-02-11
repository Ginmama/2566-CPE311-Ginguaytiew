#include "stm32l1xx.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_utils.h"
#include "VL53L0X.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_i2c.h"

// ????? I2C ????????????
#define null = null;
#define I2C_ADDRESS 0x29
#define I2C_TIMEOUT 1000

statInfo_t *info;

// Function prototypes
void i2c_init(void);
void i2c_write(uint8_t address, uint8_t* data, uint8_t length);
void i2c_read(uint8_t address, uint8_t* data, uint8_t length);
void delay_ms(uint32_t ms);

int main(void)
{
  // Initialize I2C
  i2c_init();

  // Initialize VL53L0X sensor
  initVL53L0X(0); // ??????? 1V8

  // Main loop
  while (1)
  {
    // ??????????????? Single Measurement
    // Read distance from sensor with provided address
		uint16_t distance_mm = readRangeSingleMillimeters(info);

    delay_ms(1000);
  }
}

// I2C initialization
void i2c_init(void)
{
  // ??????? GPIO ???? I2C
  LL_GPIO_InitTypeDef gpio_init_struct;

  // SDA (PB3)
  gpio_init_struct.Pin = LL_GPIO_PIN_3;
  gpio_init_struct.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init_struct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init_struct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  gpio_init_struct.Pull = LL_GPIO_PULL_UP;
  gpio_init_struct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &gpio_init_struct);

  // SCL (PB8)
  gpio_init_struct.Pin = LL_GPIO_PIN_8;
  LL_GPIO_Init(GPIOB, &gpio_init_struct);

  // ??????? I2C
  LL_I2C_InitTypeDef i2c_init_struct;
  i2c_init_struct.PeripheralMode = LL_I2C_MODE_I2C;
  i2c_init_struct.OwnAddress1 = 0;
  i2c_init_struct.TypeAcknowledge = LL_I2C_ACK;
  i2c_init_struct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &i2c_init_struct);

  // ?????????? I2C
  LL_I2C_Enable(I2C1);
}


// I2C write function
void i2c_write(uint8_t address, uint8_t* data, uint8_t length)
{
  // ??????????????????????? I2C
}

// I2C read function
void i2c_read(uint8_t address, uint8_t* data, uint8_t length)
{
  // ???????????????????????? I2C
}

// Delay function
void delay_ms(uint32_t ms)
{
  // ???????????????? (delay) ??????????
}
