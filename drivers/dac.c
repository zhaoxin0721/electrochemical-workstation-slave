#include <nrf.h>
//#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include <app_error.h>

//SPI实例定义
#define SPI_INSTANCE 0
#define const nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);


//DAC配置
#define DAC8554_RESOLUTION 65535UL // 16位DAC分辨率
// SPI引脚定义
#define SPI_SCK_PIN  25 // SPI时钟引脚
#define SPI_MOSI_PIN 26 // SPI主输出从输入引脚
#define SPI_MISO_PIN 27 // SPI主输入从输出引脚
#define SPI_CS_PIN   28 // SPI片选引脚
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    




























#include "dac.h"
#include <nrfx_spi.h>

#define DAC_SPI_INSTANCE  0
#define DAC_CS_PIN        28 // 片选引脚，根据实际连接修改

static const nrfx_spi_t spi = NRFX_SPI_INSTANCE(DAC_SPI_INSTANCE);

int dac_init(void) {
    nrfx_spi_config_t spi_config = {
        .sck_pin      = 25, // SPI SCK
        .mosi_pin     = 26, // SPI MOSI
        .miso_pin     = NRFX_SPI_PIN_NOT_USED,
        .ss_pin       = DAC_CS_PIN,
        .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc          = 0xFF,
        .frequency    = NRF_SPI_FREQ_1M,
        .mode         = NRF_SPI_MODE_0,
        .bit_order    = NRF_SPI_BIT_ORDER_MSB_FIRST,
    };
    return nrfx_spi_init(&spi, &spi_config, NULL, NULL);
}

int dac_write(uint16_t value) {
    // MCP4921: 12位数据，左对齐，控制字节
    uint8_t tx_buf[2];
    tx_buf[0] = 0x30 | ((value >> 8) & 0x0F); // 控制字节+高4位
    tx_buf[1] = value & 0xFF;                 // 低8位
    nrfx_spi_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length   = 2,
        .p_rx_buffer = NULL,
        .rx_length   = 0
    };
    return nrfx_spi_xfer(&spi, &xfer, 0);
}