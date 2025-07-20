#include "nrf52_peripheral.h"
#include <zephyr/drivers/adc.h> // nRF52 ADC 驱动头文件
// #include "nrf_drv_dac.h" // 如需 DAC
// #include "nrf_gpio.h"    // 如需 GPIO
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#define ADC_NODE        DT_NODELABEL(adc) // 适用于大多数 nRF52 板卡
#define ADC_RESOLUTION  12
#define ADC_CHANNEL_ID  1 // 你的ADC通道号
#define ADC_INPUT       NRF_SAADC_INPUT_AIN1 // 你的ADC输入引脚

#define DAC_SPI_NODE DT_NODELABEL(spi1) // 你的 SPI 实例
#define DAC_CS_PIN   15 // 片选引脚
#define DAC_LOAD_PIN 21 // DAC_LOAD
#define DAC_ENA_PIN  22 // DAC_ENA
#define GPIO_PORT    DT_LABEL(DT_NODELABEL(gpio0))

// ADC 初始化（只需初始化一次）
static void saadc_init(void)
{
    static int initialized = 0;
    if (initialized) return;
    // nrf_drv_saadc_init(NULL, NULL); // 删除
    // nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); // 删除
    // nrf_drv_saadc_channel_init(0, &channel_config); // 删除
    initialized = 1;
}

// 替换原 AD5940_ReadADC，实际用 nRF52 的 ADC
int AD5940_ReadADC(void)
{
    const struct device *adc_dev = device_get_binding("ADC_0");
    if (!adc_dev) {
        return -1;
    }

    struct adc_channel_cfg channel_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = ADC_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
        .input_positive = ADC_INPUT,
#endif
    };

    adc_channel_setup(adc_dev, &channel_cfg);

    int16_t sample_buffer;
    struct adc_sequence sequence = {
        .channels = BIT(ADC_CHANNEL_ID),
        .buffer = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = ADC_RESOLUTION,
    };

    int ret = adc_read(adc_dev, &sequence);
    if (ret) {
        return -1;
    }
    return sample_buffer;
}

// 你可以继续实现其它接口，如 DAC、GPIO、定时器等
//DAC A通道为例子 
//原来AD5940的DAC相关函数，全部改为SPI控制DAC8554。
//用Zephyr的SPI API实现。
static const struct spi_config spi_cfg = {
    .frequency = 1000000, // 1MHz，按需调整
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL, // 如果用硬件CS，这里可以配置
};

void DAC8554_Write(uint8_t channel, uint16_t value)
{
    const struct device *spi_dev = device_get_binding("SPI_1");
    const struct device *gpio_dev = device_get_binding("GPIO_0");

    if (!device_is_ready(spi_dev) || !gpio_dev) {
        printk("SPI or GPIO device not ready!\n");
        return;
    }

    // 假设需要3字节
    uint8_t tx_buf[3];
    tx_buf[0] = (channel << 6) | ((value >> 8) & 0x0F); // 命令+高4位
    tx_buf[1] = value & 0xFF; // 低8位
    tx_buf[2] = 0; // 可能是 don't care 或其它命令位
    struct spi_buf buf = { .buf = tx_buf, .len = 3 };
    struct spi_buf_set tx = { .buffers = &buf, .count = 1 };

    gpio_pin_set(gpio_dev, DAC_CS_PIN, 0);
    spi_write(spi_dev, &spi_cfg, &tx);
    gpio_pin_set(gpio_dev, DAC_CS_PIN, 1);

    gpio_pin_set(gpio_dev, DAC_LOAD_PIN, 1);
    k_sleep(K_USEC(1));
    gpio_pin_set(gpio_dev, DAC_LOAD_PIN, 0);

    gpio_pin_set(gpio_dev, DAC_ENA_PIN, 1);
    k_sleep(K_USEC(1));
    gpio_pin_set(gpio_dev, DAC_ENA_PIN, 0);
}

void set_dac_load(int value)
{
    const struct device *gpio_dev = device_get_binding("GPIO_0");
    gpio_pin_configure(gpio_dev, DAC_LOAD_PIN, GPIO_OUTPUT);
    gpio_pin_set(gpio_dev, DAC_LOAD_PIN, value);
}