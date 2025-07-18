#include "nrf.h"

#define AD5940_PIN_CS           5
#define AD5940_PIN_CLK          6
#define AD5940_PIN_MOSI         12
#define AD5940_PIN_MISO         8
#define AD5940_RESET            1
#define AD5940_GPIO0            3
#define AD5940_GPIO7            0
#define AD5940_TEST_CS              11

int  ad5940_spi_init();

int  ad5940_enable_int();
