#include "main.h"
#include "optics.h"
#include "interface_config.h"
#include "utils.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BUFFER_SIZE 128

/* internal: basic handle validation */
static inline bool adc_handle_valid(const MCP3462_Handle *a) {
    return (a && a->hspi && a->cs_port);
}
static inline bool dac_handle_valid(const MCP4922_Handle *d) {
    return (d && d->hspi && d->cs_port);
}

static const OpticsHwDesc* hw = NULL;
static uint8_t OpticsCount = 0;

/* Weak default: links even if the product doesn’t provide a mapping */
__attribute__((weak))
const OpticsHwDesc* OpticsConfig(void) {
	static const OpticsHwDesc null_config = {
		.count = 0,
		.map = NULL
	};

	return &null_config;
}

HAL_StatusTypeDef optics_adcStartConversion(int optic_index) {
    if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    if(dev->enOneshot){
    	return MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
    }

    return HAL_OK;
}

HAL_StatusTypeDef optics_startLaser(int optic_index, uint16_t power) {
    if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];

    dev->dacValue = (power > 100) ? 100 : power;
    dev->dacValue = (dev->dacValue * 4095) / 100;
    return MCP4922_WriteRaw(&dev->dac_handle,
    					   (MCP4922_Channel)optic_index,
                           MCP4922_BUF_OFF,
                           MCP4922_GAIN_1X,
                           MCP4922_ACTIVE,
                           dev->dacValue);
#if 0
    uint32_t fs_mV = dev->dac_handle.vref_mV;
    uint32_t out_mV = (uint32_t)dev->dacValue * fs_mV / 100u;


    return MCP4922_WritemV(&dev->dac_handle,
                           MCP4922_CH_A,
                           MCP4922_BUF_ON,
                           MCP4922_GAIN_1X,
                           MCP4922_ACTIVE,
                           out_mV);
#endif

    delay_us(OPTICS_SAMPLE_PERIOD_MS*1000);
}

HAL_StatusTypeDef optics_stopLaser(int optic_index) {
    if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    dev->dacValue = 0;
    return MCP4922_WriteRaw(&dev->dac_handle,
    					   (MCP4922_Channel)optic_index,
                           MCP4922_BUF_OFF,
                           MCP4922_GAIN_1X,
                           MCP4922_ACTIVE,
                           0);
    delay_us(OPTICS_SAMPLE_PERIOD_MS*1000);
}

static HAL_StatusTypeDef initialize_optic_device(int optic_index) {
	HAL_StatusTypeDef st = HAL_OK;
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
        return HAL_ERROR;
    }
    
    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];

    /* Validate wiring/handles before touching hardware */
    if (!adc_handle_valid(&dev->adc_handle)) {
        return HAL_ERROR;
    }
    if (!dac_handle_valid(&dev->dac_handle)) {
        return HAL_ERROR;
    }

    /* require a nonzero VREF for mV helper usage */
    if (dev->dac_handle.vref_mV == 0u) {
        return HAL_ERROR;
    }    

    /* Bring up devices */

    st  = MCP4922_Init(&dev->dac_handle);
    if (st != HAL_OK) {
    	return st;
    }

    st = MCP3462_Init(&dev->adc_handle);
    if (st != HAL_OK) {
    	return st;
    }

	uint8_t conv_type = dev->enOneshot?MCP3462_CONV_1SHOT_STBY:MCP3462_CONV_CONT;

	// ---- SCAN example: CH0 and CH1 single-ended ----
	MCP3462_ScanConfig scan_cfg = {
		.scan_mask    =  MCP3462_SCAN_CH0_SE | MCP3462_SCAN_CH1_SE,
		.dly_clocks   = 0,   // no extra delay between channels
		.timer_clocks = 0    // no extra delay between SCAN cycles
	};


	st = MCP3462_ConfigScan(&dev->adc_handle,
							  MCP3462_OSR_256,
							  MCP3462_GAIN_1,
							  conv_type,
							  &scan_cfg);

    if (st != HAL_OK) {
    	printf("+++++++> Failed setting configscan\r\n");
    	return st;
    }

	uint8_t  buf[BUFFER_SIZE] = {0};
	MCP3462_DumpRegs(&dev->adc_handle, buf, BUFFER_SIZE);


    /* Clear capture buffer */
	memset(dev->adcSamples, 0, ADC_UART_BUFFER_SIZE);
	dev->dataPtr = 0;

    return HAL_OK;

    /* original setup of device 
        dev->adcTx[0] = 0x78; // Reset
        adcTransfer(dev, 1);

        dev->adcTx[0] = 0x68; // Start Conversion
        adcTransfer(dev, 1);

        dev->adcTx[0] = 0x46; // Write to CONFIG0 and CONFIG1
        dev->adcTx[1] = 0x63; // CONFIG0 value
        adcTransfer(dev, 2);

        dev->adcTx[0] = 0x56; //
        dev->adcTx[1] = 0x07; //
        adcTransfer(dev, 2);

        dev->adcTx[0] = 0x47; //
        adcTransfer(dev, 1);
    */
}

HAL_StatusTypeDef optics_init() {
    
    hw = OpticsConfig();

    if (!hw || !hw->map || hw->count == 0) {
        return HAL_ERROR;
    }

    OpticsCount = hw->count;
    for (uint8_t i = 0; i < OpticsCount; ++i) {
        HAL_StatusTypeDef st = initialize_optic_device(i);
        if (st != HAL_OK) {
            return st;
        }
    }

    return HAL_OK;
}

void optics_clearBuffer(int optic_index) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	memset(dev->adcSamples, 0, ADC_UART_BUFFER_SIZE);
	dev->dataPtr = 0;
}

uint16_t optics_getSize(int optic_index) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return 0;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	return dev->dataPtr;
}

uint8_t* optics_getBuffer(int optic_index) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return NULL;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	return dev->adcSamples;
}

HAL_StatusTypeDef optics_adcReadSamples(int optic_index) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return 0;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    HAL_StatusTypeDef st = HAL_OK;

    uint8_t ch_id;
    int32_t code32;
    uint16_t raw_ch0 = 0;
    uint16_t raw_ch1 = 0;
    bool got_ch0 = false;
    bool got_ch1 = false;

	// In continuous mode, conversions happen automatically
	// No need to trigger, just read the data

	// In SCAN mode with 2 channels, we need to read exactly 2 samples per conversion cycle
	// Try up to 8 times to get both channels (allows for retries if data not ready)
	for(int i = 0; i < 8 && !(got_ch0 && got_ch1); i++){
		st = MCP3462_ReadScanSample(&dev->adc_handle, &ch_id, &code32);
		if (st == HAL_OK) {

			// code32 now holds a signed 16-bit ADC code in its low 16 bits
			uint16_t code16 = (uint16_t)code32;

            if (ch_id == 0 && !got_ch0) {
				raw_ch0 = code16;
				got_ch0 = true;
			} else if (ch_id == 1 && !got_ch1) {
				raw_ch1 = code16;
				got_ch1 = true;
			} else {
				// other channels / internal sources, ignore for now
			}

		} else if (st != HAL_BUSY) {
			return st;
		} else {
			printf("  Read attempt %d: BUSY\r\n", i);
		}

		// Small delay between read attempts
		if (!(got_ch0 && got_ch1)) {
			delay_us(500);
		}
	}

	// Store the samples we got
	dev->adcSamples[dev->dataPtr++] = (uint8_t)(raw_ch0 >> 8);
	if (dev->dataPtr >= ADC_UART_BUFFER_SIZE) dev->dataPtr = 0;
	dev->adcSamples[dev->dataPtr++] = (uint8_t)(raw_ch0 & 0xFF);
	if (dev->dataPtr >= ADC_UART_BUFFER_SIZE) dev->dataPtr = 0;

	dev->adcSamples[dev->dataPtr++] = (uint8_t)(raw_ch1 >> 8);
	if (dev->dataPtr >= ADC_UART_BUFFER_SIZE) dev->dataPtr = 0;
	dev->adcSamples[dev->dataPtr++] = (uint8_t)(raw_ch1 & 0xFF);
	if (dev->dataPtr >= ADC_UART_BUFFER_SIZE) dev->dataPtr = 0;

	return HAL_OK;
}

int optics_getDeviceCount(void) {
    return (int)OpticsCount;
}
