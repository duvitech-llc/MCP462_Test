#ifndef INC_OPTICS_H_
#define INC_OPTICS_H_

#include "main.h"
#include "mcp3462.h"
#include "mcp4922.h"
#include "stdint.h"
#include "stdbool.h"

#define MAX_DAC_CHANNELS			2   // max DAC channels
#define MAX_ADC_CHANNELS			8   // max ADC channels
#define ADC_UART_BUFFER_SIZE		256   // max 128 samples before rollover 128 * 2 bytes per sample
#define OPTICS_CAPTURE_FREQUENCY	1000    // (1MHz / 1000 = 1kHz) -> 1ms

typedef struct {
    MCP4922_Handle dac_handle;
    MCP3462_Handle adc_handle;

    uint8_t adcSamples[MAX_ADC_CHANNELS][ADC_UART_BUFFER_SIZE];
	uint16_t dataPtr[MAX_ADC_CHANNELS];

	bool enOneshot;
    uint16_t scan_bits;

    volatile uint16_t dacValue;
} OpticsDevice;

typedef struct {
    uint8_t count;
    OpticsDevice* map;
} OpticsHwDesc;

HAL_StatusTypeDef optics_init();
HAL_StatusTypeDef optics_startLaser(int optic_index, uint16_t power);
HAL_StatusTypeDef optics_stopLaser(int optic_index);
HAL_StatusTypeDef optics_adcReadSamples(int optic_index);
HAL_StatusTypeDef optics_adcStartConversion(int optic_index);
void optics_clearBuffer(int optic_index, uint8_t ch_id);
void optics_clearBuffers(int optic_index);
uint8_t* optics_getBuffer(int optic_index, uint8_t ch_id);
uint16_t optics_getSize(int optic_index, uint8_t ch_id);
int optics_getDeviceCount(void);

#endif /* INC_OPTICS_H_ */
