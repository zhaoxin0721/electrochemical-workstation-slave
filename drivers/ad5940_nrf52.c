#include "ad594x.h"
#include "ad5940_nrf52.h"

#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>

// #include <nrfx_spim.h>
#include <nrfx_spi.h>

#include <nrfx_gpiote.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad5940, LOG_LEVEL_INF);

// SPI

static nrfx_spi_t spi = NRFX_SPI_INSTANCE(0);

volatile static uint16_t    ucInterrupted = 0;      /* Flag to indicate interrupt occurred */
volatile static uint16_t    ucIntCounter  = 0;      /* Repeat number */

int  nrf52_spi_init()     // init pin 已更改-nrf52
{
  // gpio: reset
  nrf_gpio_cfg_output(AD5940_PIN_CS);
  nrf_gpio_cfg_output(AD5940_RESET);
  nrf52_CsSet();
  nrf52_RstSet();
  // spi
  nrfx_spi_config_t spi_config = NRFX_SPI_DEFAULT_CONFIG(
  AD5940_PIN_CLK,
  AD5940_PIN_MOSI,
  AD5940_PIN_MISO,
  AD5940_TEST_CS);
  spi_config.frequency = NRF_SPI_FREQ_2M;
  nrfx_err_t err = nrfx_spi_init(&spi, &spi_config, NULL, NULL);
  if (err != NRFX_SUCCESS) {
    LOG_ERR("nrfx_spi_init() failed: 0x%08x", err);
    return err;
  }
  return 0;
}

void      AD5940_CsClr(void)    // set -> low
{
    nrf_gpio_pin_clear(AD5940_PIN_CS);
}
void      nrf52_CsSet(void)    // set -> high
{
    nrf_gpio_pin_set(AD5940_PIN_CS);
}
void      AD5940_RstClr(void)   // reset -> low
{
    nrf_gpio_pin_clear(AD5940_RESET);
}
void      AD5940_RstSet(void)   // reset -> high
{
    nrf_gpio_pin_set(AD5940_RESET);
}
void      AD5940_Delay10us(uint32_t time)
{
    if (time == 0)
    {
        k_usleep(1);
    }
    else
    {
        k_usleep(10 * time);
    }
}

void      AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length)
{
	nrfx_spi_xfer_desc_t xfer_desc = NRFX_SPI_SINGLE_XFER(pSendBuffer, length, pRecvBuff, length);
	int err = nrfx_spi_xfer(&spi, &xfer_desc, 0);
	if (err != NRFX_SUCCESS) {
		// LOG_ERR("nrfx_spi_xfer() failed: 0x%08x", err);
		return;
	}
  k_usleep(10);
}

uint32_t  AD5940_GetMCUIntFlag(void)
{
    if (ucInterrupted == 0)
    {
        ucIntCounter++;
        k_usleep(10);
        if (ucIntCounter > 10000)  // 1 sec.
        {
            ucIntCounter  = 0;       // clear.
            ucInterrupted = 1;      // reset.
            return 1;
        }
    }
    // interrupt flag
    return ucInterrupted;
}
uint32_t  AD5940_ClrMCUIntFlag(void)
{
    // interrupt flag
    ucIntCounter  = 0;
    ucInterrupted = 0;
    return 1;
}

// void in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
// {
//     if (pin == AD5940_INT && nrf_gpio_pin_read(AD5940_INT) == 0) // debounce.
//     {
//         ucInterrupted = 1;
//     }
// }
/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SWX_NODE	DT_ALIAS(sw3)
#if !DT_NODE_HAS_STATUS(SWX_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SWX_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

void ad594x_gpio_in_pin_handler(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	// LOG_INF("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
  ucInterrupted = 1;
}

int ad5940_enable_int()
{
	int ret;

	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Error: button device %s is not ready",
		       button.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure %s pin %d",
		       ret, button.port->name, button.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_FALLING);
	if (ret != 0) {
		LOG_ERR("Error %d: failed to configure interrupt on %s pin %d",
			ret, button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&button_cb_data, ad594x_gpio_in_pin_handler, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	LOG_INF("Set up button at %s pin %d", button.port->name, button.pin);
  return 0;
}
