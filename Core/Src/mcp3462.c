#include "mcp3462.h"
#include <string.h>
#include <stdio.h>

/* ---------- GPIO helpers ---------- */
#define CS_LOW(d)   HAL_GPIO_WritePin((d)->cs_port, (d)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(d)  HAL_GPIO_WritePin((d)->cs_port, (d)->cs_pin, GPIO_PIN_SET)

/* ---------- SPI core ---------- */
static HAL_StatusTypeDef txrx(MCP3462_Handle *dev, uint8_t *tx, uint8_t *rx, uint16_t n) {
    return HAL_SPI_TransmitReceive(dev->hspi, tx, rx, n, HAL_MAX_DELAY);
}
static inline uint8_t CMD(MCP3462_Handle *dev, uint8_t addr, MCP3462_CmdType t) {
    return (uint8_t)(((dev->dev_addr & 0x3) << 6) | ((addr & 0xF) << 2) | (t & 0x3));
}

/* ---------- Public API ---------- */
HAL_StatusTypeDef MCP3462_FastCommand(MCP3462_Handle *dev, MCP3462_FastCmd fc) {
    if (!dev) return HAL_ERROR;
    uint8_t c = CMD(dev, (uint8_t)fc, MCP3462_CMDTYPE_FAST);   // <-- was MCP3462_CMD_FAST
    uint8_t dummy;
    CS_LOW(dev);
    HAL_StatusTypeDef st = txrx(dev, &c, &dummy, 1);
    CS_HIGH(dev);
    return st;
}

HAL_StatusTypeDef MCP3462_WriteReg(MCP3462_Handle *dev, MCP3462_Reg r, const uint8_t *data, uint8_t len) {
    if (!dev || !data || !len) return HAL_ERROR;
    uint8_t c = CMD(dev, (uint8_t)r, MCP3462_CMDTYPE_STATIC_WR); // <-- STATIC_WR (10b)
    uint8_t status;
    CS_LOW(dev);
    HAL_StatusTypeDef st = txrx(dev, &c, &status, 1); // clocks out STATUS
    if (st == HAL_OK) st = HAL_SPI_Transmit(dev->hspi, (uint8_t*)data, len, HAL_MAX_DELAY);
    CS_HIGH(dev);
    return st;
}

HAL_StatusTypeDef MCP3462_ReadReg(MCP3462_Handle *dev, MCP3462_Reg r, uint8_t *data, uint8_t len) {
    if (!dev || !data || !len) return HAL_ERROR;
    uint8_t c = CMD(dev, (uint8_t)r, MCP3462_CMDTYPE_STATIC_RD); // <-- was MCP3462_CMD_STATIC_RD
    uint8_t status;
    CS_LOW(dev);
    HAL_StatusTypeDef st = txrx(dev, &c, &status, 1); // STATUS shifts out here
    if (st == HAL_OK) st = HAL_SPI_Receive(dev->hspi, data, len, HAL_MAX_DELAY);
    CS_HIGH(dev);
    return st;
}

HAL_StatusTypeDef MCP3462_Init(MCP3462_Handle *dev) {
    if (!dev || !dev->hspi || !dev->cs_port) return HAL_ERROR;
    CS_HIGH(dev);

    /* Full reset → unlock map (LOCK=0xA5) per datasheet */
    HAL_StatusTypeDef st = MCP3462_FastCommand(dev, MCP3462_FC_FULL_RESET);
    if (st != HAL_OK) return st;
    HAL_Delay(1);

    uint8_t key = 0xA5;
    return MCP3462_WriteReg(dev, MCP3462_REG_LOCK, &key, 1);
}


/* Helper to build CONFIG0 (normal operation; leaves bit 6 = 0) */
static inline uint8_t mcp346x_build_config0(mcp346x_vref_sel_t vref,
                                            mcp346x_clk_sel_t clk,
                                            mcp346x_cs_sel_t cs,
                                            mcp346x_adc_mode_t mode)
{
    uint8_t cfg0 = 0u;

    cfg0 |= vref;     // bit 7
    /* bit 6 left 0 for normal operation */
    // cfg0 |= MCP346X_CFG0_RESERVED_BIT;
    cfg0 |= clk;      // bits 5-4
    cfg0 |= cs;       // bits 3-2
    cfg0 |= mode;     // bits 1-0

    return cfg0;
}

HAL_StatusTypeDef MCP3462_ConfigSimple(MCP3462_Handle *dev,
                                       MCP3462_OSR osr, MCP3462_Gain gain,
                                       MCP3462_DataFmt fmt, MCP3462_ConvMode mode,
                                       uint8_t vinp, uint8_t vinn)
{
    if (!dev) return HAL_ERROR;

    /* CONFIG0: internal RC clock, burnout OFF, ADC in SHDN (we'll start via fast cmd) */
    uint8_t cfg0 = mcp346x_build_config0(MCP346X_VREF_EXT, MCP346X_CLK_INT_NO_OUT, MCP346X_CS_NONE, MCP346X_ADC_CONVERSION);
    printf("Set CFG0: 0x%02X\r\n", cfg0);
	HAL_StatusTypeDef st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG0, &cfg0, 1);

    if (st != HAL_OK) return st;

    /* CONFIG1: PRE=0, OSR as requested */
    uint8_t cfg1 = (0u << MCP3462_CONFIG1_PRE_SHIFT) | ((uint8_t)osr << MCP3462_CONFIG1_OSR_SHIFT)| ((uint8_t)0x00 << 0);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG1, &cfg1, 1);
    if (st != HAL_OK) return st;

    /* CONFIG2: BOOST=1x, gain, AZ_MUX=1 for lower offset/noise */
    // CONFIG2: BOOST=1x, gain, AZ_MUX=0 (for bring-up), RESERVED=11
    uint8_t cfg2 = (0x0u << MCP3462_CONFIG2_BOOST_SHIFT)
                 | ((uint8_t)gain << MCP3462_CONFIG2_GAIN_SHIFT)
                 | (1u << MCP3462_CONFIG2_AZ_MUX_BIT)
                 | 0x03;  // RESERVED bits [1:0] must be '11'
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG2, &cfg2, 1);
    if (st != HAL_OK) return st;

    /* CONFIG3: conversion mode + data format, CRC off by default */
    uint8_t cfg3 = ((uint8_t)mode << MCP3462_CONFIG3_CONVMODE_SHIFT) | ((uint8_t)fmt << MCP3462_CONFIG3_DATAFMT_SHIFT);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG3, &cfg3, 1);
    if (st != HAL_OK) return st;

    /* MUX: VIN+ / VIN- */
    uint8_t mux = (uint8_t)((vinp & 0xF) << 4) | (uint8_t)(vinn & 0xF);
    st = MCP3462_WriteReg(dev, MCP3462_REG_MUX, &mux, 1);
    if (st != HAL_OK) return st;

    /* Clear any latched IRQ flags */
    uint8_t irq;
    MCP3462_ReadReg(dev, MCP3462_REG_IRQ, &irq, 1);

    /* Start conversions */
    return MCP3462_FastCommand(dev, MCP3462_FC_CONV_START);
}

HAL_StatusTypeDef MCP3462_ReadData16_INC(MCP3462_Handle *dev, int16_t *out)
{
    if (!dev || !out) return HAL_ERROR;

    uint8_t cmd = (uint8_t)(((dev->dev_addr & 0x3) << 6)
                   | ((MCP3462_REG_ADCDATA & 0xF) << 2)
                   | MCP3462_CMDTYPE_INC_RD);

    uint8_t rx[3] = {0};
    uint8_t tx0 = cmd;

    CS_LOW(dev);
    // Byte 0: send command, receive STATUS
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &tx0, &rx[0], 1, HAL_MAX_DELAY);
    if (st == HAL_OK) {
        // Bytes 1–2: the 16-bit data (MSB then LSB)
        st = HAL_SPI_Receive(dev->hspi, &rx[1], 2, HAL_MAX_DELAY);
    }
    CS_HIGH(dev);
    if (st != HAL_OK) return st;

    // DRDY mirror is bit6 (0 = ready)
    bool ready = ((rx[0] & 0x40u) == 0u);
    if (!ready) return HAL_BUSY;

    *out = (uint16_t)((rx[1] << 8) | rx[2]);
    return HAL_OK;
}

HAL_StatusTypeDef MCP3462_ReadData16(MCP3462_Handle *dev, int16_t *code) {
    if (!dev || !code) return HAL_ERROR;
    uint8_t buf[2];
    HAL_StatusTypeDef st = MCP3462_ReadReg(dev, MCP3462_REG_ADCDATA, buf, 2);
    if (st == HAL_OK) {
        *code = (int16_t)((buf[0] << 8) | buf[1]);
    }
    return st;
}

HAL_StatusTypeDef MCP3462_ReadData32_INC(MCP3462_Handle *dev, int32_t *out)
{
    uint8_t cmd = (uint8_t)(((dev->dev_addr & 0x3) << 6)
                   | ((MCP3462_REG_ADCDATA & 0xF) << 2)
                   | MCP3462_CMDTYPE_INC_RD);
    uint8_t rx[5] = {0};
    CS_LOW(dev);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi, &cmd, &rx[0], 1, HAL_MAX_DELAY); // STATUS
    if (st == HAL_OK) st = HAL_SPI_Receive(dev->hspi, &rx[1], 4, HAL_MAX_DELAY);               // 4 data bytes
    CS_HIGH(dev);
    if (st != HAL_OK) return st;
    if ((rx[0] & 0x40u) != 0u) return HAL_BUSY;

    *out = (int32_t)((uint32_t)rx[1]<<24 | (uint32_t)rx[2]<<16 | (uint32_t)rx[3]<<8 | (uint32_t)rx[4]);
    return HAL_OK;
}

HAL_StatusTypeDef MCP3462_ReadData32(MCP3462_Handle *dev, int32_t *out) {
    if (!dev || !out) return HAL_ERROR;
    uint8_t b[4];
    HAL_StatusTypeDef st = MCP3462_ReadReg(dev, MCP3462_REG_ADCDATA, b, 4);
    if (st == HAL_OK) {
        *out = (int32_t)((uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | (uint32_t)b[3]);
    }
    return st;
}

/* Ready checks */
bool MCP3462_DataReadyIRQ(MCP3462_Handle *dev) {
    if (!dev || !dev->irq_port) return false;
    return (HAL_GPIO_ReadPin(dev->irq_port, dev->irq_pin) == GPIO_PIN_RESET);
}

bool MCP3462_DataReadyStatus(MCP3462_Handle *dev) {
    if (!dev) return false;
    uint8_t c = CMD(dev, (uint8_t)MCP3462_REG_ADCDATA, MCP3462_CMDTYPE_STATIC_RD);
    uint8_t status=0;
    CS_LOW(dev);
    if (txrx(dev, &c, &status, 1) != HAL_OK) { CS_HIGH(dev); return false; }
    CS_HIGH(dev);
    /* DR bit is 0 when data is ready */
    return (status & 0x80u) == 0u;
}

HAL_StatusTypeDef MCP3462_DumpRegs(MCP3462_Handle *dev, uint8_t *buf, uint8_t buflen) {
    if (!buf || buflen < 8) return HAL_ERROR;
    uint8_t off = 0; uint8_t v;
    
    printf("MCP3462 Register Dump:\r\n");
    printf("=====================\r\n");
    
    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG0, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG0: 0x%02X\r\n", v);
    } else {
        printf("CONFIG0: READ FAILED\r\n");
    }
    
    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG1, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG1: 0x%02X\r\n", v);
    } else {
        printf("CONFIG1: READ FAILED\r\n");
    }
    
    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG2, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG2: 0x%02X\r\n", v);
    } else {
        printf("CONFIG2: READ FAILED\r\n");
    }
    
    if (MCP3462_ReadReg(dev, MCP3462_REG_CONFIG3, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("CONFIG3: 0x%02X\r\n", v);
    } else {
        printf("CONFIG3: READ FAILED\r\n");
    }
    
    if (MCP3462_ReadReg(dev, MCP3462_REG_MUX, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("MUX:     0x%02X\r\n", v);
    } else {
        printf("MUX:     READ FAILED\r\n");
    }
    
    if (MCP3462_ReadReg(dev, MCP3462_REG_IRQ, &v, 1) == HAL_OK) {
        buf[off++] = v;
        printf("IRQ:     0x%02X\r\n", v);
    } else {
        printf("IRQ:     READ FAILED\r\n");
    }
    
    printf("=====================\r\n");
    return HAL_OK;
}

int8_t MCP3462_(MCP3462_Handle *dev) {
    uint8_t saved = dev->dev_addr;
    for (uint8_t a = 0; a < 4; ++a) {
    	dev->dev_addr = a;
        uint8_t rb = 0xFF;

        // Try to unlock at this address
        uint8_t key = 0xA5;
        MCP3462_WriteReg(dev, MCP3462_REG_LOCK, &key, 1);
        HAL_Delay(1);
        MCP3462_ReadReg(dev, MCP3462_REG_LOCK, &rb, 1);

        // Heuristic: if readback is neither 0xFF nor the hard-reset default (often 0x00),
        // or if subsequent CFG0 write sticks, this is our address.
        uint8_t cfg0 = 0xC0, cfg0_rb=0xFF;
        MCP3462_WriteReg(dev, MCP3462_REG_CONFIG0, &cfg0, 1);
        MCP3462_ReadReg(dev, MCP3462_REG_CONFIG0, &cfg0_rb, 1);

        if (cfg0_rb == cfg0) {
            return (int8_t)a;
        }
    }
    dev->dev_addr = saved;
    return -1;
}

HAL_StatusTypeDef MCP3462_ConfigScan(MCP3462_Handle *dev,
                                     MCP3462_OSR   osr,
                                     MCP3462_Gain  gain,
                                     const MCP3462_ScanConfig *scan_cfg)
{
    if (!dev || !scan_cfg) return HAL_ERROR;

    HAL_StatusTypeDef st;

    /* CONFIG0: internal RC, no burnout, ADC in CONVERSION mode */
    uint8_t cfg0 = mcp346x_build_config0(
        MCP346X_VREF_EXT,           // or MCP346X_VREF_INT_2V4_BUF if you want internal ref
        MCP346X_CLK_INT_NO_OUT,     // internal clock, no CLK output
        MCP346X_CS_NONE,            // no bias current
        MCP346X_ADC_CONVERSION      // ADC_MODE = 11 (conversion)
    );
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG0, &cfg0, 1);
    if (st != HAL_OK) return st;

    /* CONFIG1: PRE=0, OSR as requested */
    uint8_t cfg1 = (0u << MCP3462_CONFIG1_PRE_SHIFT)
                 | ((uint8_t)osr << MCP3462_CONFIG1_OSR_SHIFT);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG1, &cfg1, 1);
    if (st != HAL_OK) return st;

    /* CONFIG2: BOOST=1x, gain, AZ_MUX=1, RESERVED[1:0]=11 */
    uint8_t cfg2 = (0x0u << MCP3462_CONFIG2_BOOST_SHIFT)
                 | ((uint8_t)gain << MCP3462_CONFIG2_GAIN_SHIFT)
                 | (1u << MCP3462_CONFIG2_AZ_MUX_BIT)
                 | 0x03u; // reserved bits must be '11'
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG2, &cfg2, 1);
    if (st != HAL_OK) return st;

    /* CONFIG3: continuous SCAN cycles + 32-bit FULL format (CH_ID in MSbits) */
    uint8_t cfg3 = ((uint8_t)MCP3462_CONV_CONT   << MCP3462_CONFIG3_CONVMODE_SHIFT) |
                   ((uint8_t)MCP3462_DATAFMT_32_FULL << MCP3462_CONFIG3_DATAFMT_SHIFT);
    st = MCP3462_WriteReg(dev, MCP3462_REG_CONFIG3, &cfg3, 1);
    if (st != HAL_OK) return st;

    /* SCAN: DLY[2:0] + SCAN[15:0] */
    uint32_t scan_val = (((uint32_t)(scan_cfg->dly_clocks & 0x7u)) << 21)
                      | ((uint32_t)scan_cfg->scan_mask & 0xFFFFu);
    uint8_t scan_bytes[3] = {
        (uint8_t)((scan_val >> 16) & 0xFF),
        (uint8_t)((scan_val >>  8) & 0xFF),
        (uint8_t)( scan_val        & 0xFF)
    };
    st = MCP3462_WriteReg(dev, MCP3462_REG_SCAN, scan_bytes, 3);
    if (st != HAL_OK) return st;

    /* TIMER: delay between SCAN cycles (only used when CONV_MODE=continuous) */
    uint32_t t = scan_cfg->timer_clocks & 0x00FFFFFFu;
    uint8_t timer_bytes[3] = {
        (uint8_t)((t >> 16) & 0xFF),
        (uint8_t)((t >>  8) & 0xFF),
        (uint8_t)( t        & 0xFF)
    };
    st = MCP3462_WriteReg(dev, MCP3462_REG_TIMER, timer_bytes, 3);
    if (st != HAL_OK) return st;

    /* Clear any latched IRQ flags */
    uint8_t irq_dummy;
    (void)MCP3462_ReadReg(dev, MCP3462_REG_IRQ, &irq_dummy, 1);

    /* Start / restart conversions (SCAN mode is active because SCAN[15:0] != 0) */
    return MCP3462_FastCommand(dev, MCP3462_FC_CONV_START);
}

HAL_StatusTypeDef MCP3462_ReadScanSample(MCP3462_Handle *dev,
                                         uint8_t *ch_id,
                                         int32_t *code)
{
    if (!dev || !ch_id || !code) return HAL_ERROR;

    int32_t raw32 = 0;
    HAL_StatusTypeDef st = MCP3462_ReadData32_INC(dev, &raw32);
    if (st != HAL_OK) {
        return st;  // HAL_BUSY if DRDY not yet low, etc.
    }

    /* DATAFMT = 32_FULL:
     * [31:28] = CH_ID[3:0]
     * [27:24] = 0000
     * [23:0]  = 24-bit conversion result (two's complement)
     */
    uint8_t cid = (uint8_t)((raw32 >> 28) & 0x0Fu);
    int32_t raw24 = (raw32 & 0x00FFFFFFu);

    /* sign-extend 24-bit value to 32 bits */
    if (raw24 & 0x00800000u) {
        raw24 |= 0xFF000000u;
    }

    *ch_id = cid;
    *code  = raw24;
    return HAL_OK;
}

