#include "main.h"
#include "optics.h"
#include "interface_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define USE_ONE_SHOT 1

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
    return MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
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

	HAL_Delay(OPTICS_SAMPLE_PERIOD_MS);
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
	HAL_Delay(OPTICS_SAMPLE_PERIOD_MS);
}

static HAL_StatusTypeDef initialize_optic_device(int optic_index) {
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
    HAL_StatusTypeDef st = MCP3462_Init(&dev->adc_handle);
    if (st != HAL_OK) {
    	return st;
    }

    st = MCP4922_Init(&dev->dac_handle);
    if (st != HAL_OK) {
    	return st;
    }

	HAL_Delay(5);

    /* Configure ADC and start continuous conversions */
    st = MCP3462_ConfigSimple(&dev->adc_handle,
                              MCP3462_OSR_256,
                              MCP3462_GAIN_1,
                              MCP3462_DATAFMT_16,
#if USE_ONE_SHOT
							MCP3462_CONV_1SHOT_STBY,
#else
                            MCP3462_CONV_CONT,
#endif
							  MCP3462_CH0,
							  MCP3462_AGND);  // VIN+ = CH0, VIN- = AGND
    if (st != HAL_OK) {
    	return st;
    }

    /* Clear capture buffer */
	memset(dev->adcSamples, 0, ADC_UART_BUFFER_SIZE);
	dev->dataPtr = 0;

#if USE_ONE_SHOT
	st = MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
	if (st != HAL_OK) {
		return st;
	}
#endif

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
    int16_t code;
	
	// Read 16 bits from ADCDATA
    HAL_StatusTypeDef st = MCP3462_ReadData16_INC(&dev->adc_handle, &code);
    if (st != HAL_OK) {
    	return st;
    }

    dev->adcSamples[dev->dataPtr++] = (uint8_t)(code >> 8);
    dev->adcSamples[dev->dataPtr++] = (uint8_t)(code & 0xFF);
    if (dev->dataPtr >= ADC_UART_BUFFER_SIZE) dev->dataPtr = 0;

	if (dev->dataPtr >= ADC_UART_BUFFER_SIZE) {
		dev->dataPtr = 0;
	}

#if USE_ONE_SHOT
	st = MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START);
	if (st != HAL_OK) {
		return st;
	}
#endif
	return HAL_OK;
}

int optics_getDeviceCount(void) {
    return (int)OpticsCount;
}
