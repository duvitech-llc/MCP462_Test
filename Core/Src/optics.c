#include "main.h"
#include "optics.h"
#include "interface_config.h"
#include "util.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// #define BUFFER_SIZE 128

/* internal: basic handle validation */
static inline bool adc_handle_valid(const MCP3462_Handle *a) {
    return (a && a->hspi && a->cs_port);
}
static inline bool dac_handle_valid(const MCP4922_Handle *d) {
    return (d && d->hspi && d->cs_port);
}

static OpticsHwDesc* hw = NULL;
static uint8_t OpticsCount = 0;
static uint32_t active_optics_mask = 0;
static uint32_t active_laser_mask = 0;

/* Weak default: links even if the product doesn’t provide a mapping */
__attribute__((weak))
OpticsHwDesc* OpticsConfig(void) {
	static OpticsHwDesc null_config = {
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
		.scan_mask    =  dev->scan_bits,
		.dly_clocks   = 0,   // no extra delay between channels
		.timer_clocks = 0    // no extra delay between SCAN cycles
	};


	st = MCP3462_ConfigScan(&dev->adc_handle,
							  MCP3462_OSR_32,
							  MCP3462_GAIN_1,
							  conv_type,
							  &scan_cfg);

    if (st != HAL_OK) {
    	printf("+++++++> Failed setting configscan\r\n");
    	return st;
    }

	// uint8_t  buf[BUFFER_SIZE] = {0};
	// MCP3462_DumpRegs(&dev->adc_handle, buf, BUFFER_SIZE);


    /* Clear capture buffer */
	for(int i = 0; i <MAX_ADC_CHANNELS; i++){
		memset(&dev->adcSamples[i], 0, ADC_UART_BUFFER_SIZE);
		dev->dataPtr[i] = 0;
	}

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
    
    active_optics_mask = 0;
    active_laser_mask = 0;

    return HAL_OK;
}

void optics_clearBuffers(int optic_index)
{
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    for(int i=0; i<MAX_ADC_CHANNELS; i++){
		memset(&dev->adcSamples[i], 0, ADC_UART_BUFFER_SIZE);
		dev->dataPtr[i] = 0;
    }

}

void optics_clearBuffer(int optic_index, uint8_t ch_id) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS)) {
		return;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	memset(&dev->adcSamples[ch_id], 0, ADC_UART_BUFFER_SIZE);
	dev->dataPtr[ch_id] = 0;
}

uint16_t optics_getSize(int optic_index, uint8_t ch_id) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS)) {
		return 0;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	return dev->dataPtr[ch_id];
}

uint8_t* optics_getBuffer(int optic_index, uint8_t ch_id) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS)) {
		return NULL;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	return (uint8_t*)&(dev->adcSamples[ch_id]);
}

HAL_StatusTypeDef optics_adcReadSamples(int optic_index) {
	if ((!hw) || (!hw->map) || (optic_index < 0) || ((uint8_t)optic_index >= OpticsCount)) {
		return 0;
	}

    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    HAL_StatusTypeDef st = HAL_OK;

    uint8_t ch_id;
    int32_t code32;
    bool got_ch0 = false;
    bool got_ch1 = false;

	// In continuous mode, conversions happen automatically
	// No need to trigger, just read the data

	// In SCAN mode with 2 channels, we need to read exactly 2 samples per conversion cycle
	// Try up to 8 times to get both channels (allows for retries if data not ready)
	for(int i = 0; i < 8 && !(got_ch0 && got_ch1); i++){
		st = MCP3462_ReadScanSample(&dev->adc_handle, &ch_id, &code32);
		if (st == HAL_OK) {

			printf("  ch: %d value: 0x%04X\r\n", ch_id, (uint16_t)code32);

			// code32 now holds a signed 16-bit ADC code in its low 16 bits
			uint16_t code16 = (uint16_t)code32;

			// Store the samples we got
			dev->adcSamples[ch_id][dev->dataPtr[ch_id]++] = (uint8_t)(code16 >> 8);
			if (dev->dataPtr[ch_id] >= ADC_UART_BUFFER_SIZE) dev->dataPtr[ch_id] = 0;
			dev->adcSamples[ch_id][dev->dataPtr[ch_id]++] = (uint8_t)(code16 & 0xFF);
			if (dev->dataPtr[ch_id] >= ADC_UART_BUFFER_SIZE) dev->dataPtr[ch_id] = 0;

            if (ch_id == 0) {
				got_ch0 = true;
			} else if (ch_id == 1) {
				got_ch1 = true;
			} else {
				// other channels / internal sources, ignore for now
			}

		} else if (st != HAL_BUSY) {
			return st;
		} else {
			printf("  Read attempt %d: BUSY\r\n", i);
		}

		delay_us(1200);

	}

	return HAL_OK;
}

int optics_getDeviceCount(void) {
    return (int)OpticsCount;
}

HAL_StatusTypeDef optics_adcStart(uint32_t mask)
{
	HAL_StatusTypeDef status = HAL_OK;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

	/* Only start devices that aren't already active */
	uint32_t new_mask = mask & ~active_optics_mask;

    for (uint8_t bit = 0; bit < 32; ++bit) {
        if ((new_mask & (1u << bit)) == 0) {
            continue; /* this bit not set */
        }

        uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

        if (optic_index >= OpticsCount) {
            /* mark error but continue processing other bits */
            status = HAL_ERROR;
            break;
        }

        OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	    if(dev->enOneshot) // start first conversion
	    {
	    	if(MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START) != HAL_OK)
	    	{
	    		status = HAL_ERROR;
	    	}
	    }
    }

	/* update active mask with newly started devices */
    active_optics_mask |= mask;

	return status;
}

HAL_StatusTypeDef optics_adcStop(uint32_t mask)
{
	HAL_StatusTypeDef status = HAL_OK;
    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

	/* Only stop devices that are currently active */
	uint32_t devices_to_stop = mask & active_optics_mask;

	/* Remove stopped devices from active mask */
    active_optics_mask &= ~devices_to_stop;

	// stop all devices when mask is zero
    if(active_optics_mask == 0)
    {
    	// all stopped
        for(int x=0; x<OpticsCount; x++)
        {

            OpticsDevice* dev = (OpticsDevice*)&hw->map[x];
            if(MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_STANDBY) != HAL_OK)
            {
                status = HAL_ERROR;
            }
        }
    }

	return status;
}

uint32_t optics_get_active_optics_mask()
{
	return active_optics_mask;
}

HAL_StatusTypeDef optics_adcRead()
{
	HAL_StatusTypeDef status = HAL_OK;

	if ((!hw) || (!hw->map)) {
		return HAL_ERROR;
	}

	if (active_optics_mask == 0) {
		return HAL_OK; /* nothing to read */
	}

	/* Iterate through all possible bits in the mask */
	for (uint8_t bit = 0; bit < 32; ++bit) {
		if ((active_optics_mask & (1u << bit)) == 0) {
			continue; /* this channel not active */
		}

		/* Extract optic_index and channel ID from bit position */
		uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
		uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

		/* Validate bounds */
		if (optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS) {
			status = HAL_ERROR;
			continue;
		}

		OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
		HAL_StatusTypeDef st = HAL_OK;

		uint8_t read_ch_id;
		int32_t code32;
		uint8_t channels_mask = 0;
		/* Extract the expected channels for this optic from the active_optics_mask */
		uint8_t expected_mask = (uint8_t)((active_optics_mask >> (optic_index * 8)) & 0xFF);

		for(int i = 0; i < 8 && channels_mask != expected_mask; i++){
			st = MCP3462_ReadScanSample(&dev->adc_handle, &read_ch_id, &code32);
			if (st == HAL_OK) {
				/* code32 now holds a signed 16-bit ADC code in its low 16 bits */
				uint16_t code16 = (uint16_t)code32;

				/* Store the samples we got */
				dev->adcSamples[read_ch_id][dev->dataPtr[read_ch_id]++] = (uint8_t)(code16 >> 8);
				if (dev->dataPtr[read_ch_id] >= ADC_UART_BUFFER_SIZE) dev->dataPtr[read_ch_id] = 0;
				dev->adcSamples[read_ch_id][dev->dataPtr[read_ch_id]++] = (uint8_t)(code16 & 0xFF);
				if (dev->dataPtr[read_ch_id] >= ADC_UART_BUFFER_SIZE) dev->dataPtr[read_ch_id] = 0;

				/* Mark this channel as received */
				channels_mask |= (1u << read_ch_id);

			} else if (st != HAL_BUSY) {
				status = HAL_ERROR;
				break;
			}
		}

		/* Start next conversion if in oneshot mode */
		if(dev->enOneshot) {
			if(MCP3462_FastCommand(&dev->adc_handle, MCP3462_FC_CONV_START) != HAL_OK) {
				status = HAL_ERROR;
			}
		}
	}

	return status;
}

HAL_StatusTypeDef optics_getBuffer_byMask(uint32_t mask, uint8_t** out_buffer,  uint16_t* out_size)
{
    if((!hw) || (!hw->map) || !out_buffer || !out_size || mask == 0){
        return HAL_ERROR;
    }

    /* require exactly one bit set in mask */
    if ((mask & (mask - 1)) != 0u) return HAL_ERROR;  

	/* Find which bit is set */
	uint8_t bit = 0;
	uint32_t temp_mask = mask;
	while ((temp_mask & 1) == 0) {
		temp_mask >>= 1;
		bit++;
	}

	/* Extract optic_index and channel ID from bit position */
	uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
	uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

	/* Validate that the optic_index and channel are within bounds */
	if (optic_index >= OpticsCount || ch_id >= MAX_ADC_CHANNELS) {
		return HAL_ERROR;
	}

	/* Get the device and return the buffer and size */
	OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
	

    *out_buffer = (uint8_t*)&(dev->adcSamples[ch_id]);
    *out_size = dev->dataPtr[ch_id];

	return HAL_OK;
}

HAL_StatusTypeDef optics_clearBuffer_byMask(uint32_t mask)
{
	HAL_StatusTypeDef status = HAL_OK;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

    for (uint8_t bit = 0; bit < 32; ++bit) {
        if ((mask & (1u << bit)) == 0) {
            continue; /* this bit not set */
        }

        uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
        uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

        if (optic_index >= OpticsCount) {
            /* mark error but continue processing other bits */
            status = HAL_ERROR;
            break;
        }

        // printf("Clear OPTICS IDX: %d  CH: %d\r\n", optic_index,  ch_id);
        /* call the provided clear function */
        OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];
    	memset(&dev->adcSamples[ch_id], 0, ADC_UART_BUFFER_SIZE);
    	dev->dataPtr[ch_id] = 0;
    }

	return status;
}

HAL_StatusTypeDef optics_startLaser_byMask(uint32_t mask, uint16_t power) {
	HAL_StatusTypeDef status = HAL_OK;
	uint16_t set_power_level = 0;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }

	if (mask == 0) {
		return HAL_OK; /* nothing to set */
	}

    active_laser_mask |= mask;
    set_power_level = (power > 100) ? 100 : power;

	/* Iterate through all possible bits in the mask parameter (not active_laser_mask) */
	for (uint8_t bit = 0; bit < 32; ++bit) {
		if ((mask & (1u << bit)) == 0) {
			continue; /* this channel not in the requested mask */
		}

		/* Extract optic_index and channel ID from bit position */
		uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
		uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */

	    if (optic_index >= OpticsCount) {
	    	status = HAL_ERROR;
	    	continue;
	    }

	    OpticsDevice* dev = &hw->map[optic_index];

	    if (!dev) {
	    	status = HAL_ERROR;
	    	continue;
	    }

	    uint32_t calc = (((uint32_t)set_power_level * 4095) + 50) / 100;
	    dev->dacValue = (uint16_t)calc;

	    status = MCP4922_WriteRaw(&dev->dac_handle,
	    					   (MCP4922_Channel)ch_id,
	                           MCP4922_BUF_OFF,
	                           MCP4922_GAIN_1X,
	                           MCP4922_ACTIVE,
	                           dev->dacValue);
	}

    delay_us(OPTICS_SAMPLE_PERIOD_MS*1000);
    return status;
}

HAL_StatusTypeDef optics_stopLaser_byMask(uint32_t mask) {
	HAL_StatusTypeDef status = HAL_OK;

    if ((!hw) || (!hw->map)) {
        return HAL_ERROR;
    }


	if (mask == 0) {
		return HAL_OK; /* nothing to set */
	}

	// Clear bits from the active laser mask
	active_laser_mask &= ~mask;

	/* Iterate through mask */
	for (uint8_t bit = 0; bit < 32; ++bit) {
		if ((mask & (1u << bit)) == 0) {
			continue; /* this channel not active */
		}

		/* Extract optic_index and channel ID from bit position */
		uint8_t ch_id = (uint8_t)(bit & 0x7);          /* bit % 8 */
		uint8_t optic_index = (uint8_t)(bit >> 3);     /* bit / 8 */
	    OpticsDevice* dev = (OpticsDevice*)&hw->map[optic_index];

	    dev->dacValue = 0;


	    status = MCP4922_WriteRaw(&dev->dac_handle,
	    					   (MCP4922_Channel)ch_id,
	                           MCP4922_BUF_OFF,
	                           MCP4922_GAIN_1X,
	                           MCP4922_ACTIVE,
	                           dev->dacValue);
	}
    return status;
}

uint32_t optics_get_active_laser_mask(void)
{
	return active_laser_mask;
}
