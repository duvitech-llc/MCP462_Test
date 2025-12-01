/*
 * optics_config.c — per-product hardware mapping for optics devices
 */

#include "optics.h"
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;

static OpticsDevice OPTICS_MAP[] = {
    {
		.adc_handle = {
			.cs_port = ADC_CS_GPIO_Port,
			.cs_pin = ADC_CS_Pin,
			.dev_addr = 1,
			.hspi = &hspi1
		},
		.dac_handle = {
			.cs_port = DAC_CS_GPIO_Port,
			.cs_pin = DAC_CS_Pin,
			.ldac_port = NULL,
			.ldac_pin = 0,
			.shdn_port = NULL,
			.shdn_pin = 0,
			.vref_mV = 3300,
			.hspi = &hspi1
		},
		.enOneshot = true,  // oneshot mode: trigger conversion before each read
		.scan_bits = MCP3462_SCAN_CH0_SE | MCP3462_SCAN_CH1_SE,
    }
};

/* Return descriptor for this product */
const OpticsHwDesc* OpticsConfig(void)
{
    /* Set .count to the number of active entries you actually use */
    static OpticsHwDesc optics_config = {
        .count = 1,
        .map   = OPTICS_MAP,
    };
    return &optics_config;
}
