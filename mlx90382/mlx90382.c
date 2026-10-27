/*
 * Copyright (c) 2025, Felipe Neves
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90382

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/sensor/mlx90382.h>

LOG_MODULE_REGISTER(mlx90382, CONFIG_SENSOR_LOG_LEVEL);

#define DELAY_US 2

/* The devicetree node identifier for the "cs0" alias. */
// #define CS_PIN0_NODE DT_ALIAS(cs0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// static const struct gpio_dt_spec cs0 = GPIO_DT_SPEC_GET(CS_PIN0_NODE, gpios);

#define MLX90382_REG_ANGLE        0x3FFF
#define MLX90382_READ_BIT         BIT(14)
#define MLX90382_PARITY_BIT       BIT(15)
#define MLX90382_DATA_MASK        GENMASK(13, 0)
#define MLX90382_ERROR_BIT        BIT(14)
// #define MLX90382_MAX_STEPS        0x1000	//MLX90382AA/AB
#define MLX90382_MAX_STEPS        0x10000	//MLX90382BA
#define MLX90382_FULL_ANGLE_DEG   360
#define MLX90382_MICRO_DEGREE     1000000

enum MLX90382_CMD
{
    CMD_RR  = 0xCC,
    CMD_RW  = 0x78,
    CMD_FR  = 0x34,
    CMD_SFR = 0xD0, 
};

// ADDR = 0x034
union mlx90382_spi_config
{
    uint16_t word_val;

    struct
    {
        uint8_t spi_frfs     	: 4;    //BIT0-BIT3
        uint8_t spi_frfsen   	: 1;	//BIT4
        uint8_t spi_frcrcen   	: 1;	//BIT5
        uint8_t spi_frinv     	: 4;	//BIT6-BIT9
        uint8_t spi_mode   		: 2;	//BIT10-BIT11
        uint8_t spi_dmy    		: 2;	//BIT12-BIT13
        uint8_t not_used    	: 2;	//BIT14-BIT15
    };
};

union mlx90382_spi_faddr01
{
    uint16_t word_val;

    struct
    {
        uint8_t spi_faddr0     : 8;    //BIT0-BIT7
        uint8_t spi_faddr1     : 8;    
    };
};

union mlx90382_spi_faddr23
{
    uint16_t word_val;

    struct
    {
        uint8_t spi_faddr2     : 8;    //BIT0-BIT7
        uint8_t spi_faddr3     : 8;    
    };
};

union mlx90382_config_reg
{
    uint16_t word_val;

    struct
    {
        uint8_t sensing_mode     : 3;    //BIT0-BIT2
        uint8_t gpio_if          : 2;
        uint8_t abi_if           : 1;
        uint8_t gpio_cfg         : 5;
        uint8_t abi_cfg          : 5;
    };
};

union mlx90382_application_mode_reg
{
    uint16_t word_val;

    struct
    {
        uint8_t application_mode     	: 2;    //BIT0-BIT2
        uint8_t nvram_crc_calculation   : 1;
        uint16_t not_used               : 13;
    };
};

union mlx90382_crc_calc_reg
{
    uint16_t word_val;

    struct
    {
        uint8_t crc_calc_strt     	: 1;    //BIT0-BIT2
        uint8_t crc_calc_done   	: 1;
        uint16_t not_used           : 14;
    };
};

union mlx90382_0xbe
{
    uint16_t word_val;

    struct
    {
        uint8_t de_aroc     	: 1;    //BIT0-BIT2
        uint8_t de_sr   		: 3;
        uint8_t de_die          : 1;
        uint8_t de_intp         : 1;
        uint8_t de_scxy         : 1;
        uint8_t de_dsp_rmm      : 2;
        uint8_t reserved        : 6;
        uint8_t eh_wd_en        : 1;
    };
};

// ADDR = 0x02C
union mlx90382_state
{
    uint16_t word_val;

    struct
    {
        uint8_t state     	: 8;    //BIT0 - 7
        uint8_t reserved   	: 8;
    };
};

// ADDR = 0x02E
union mlx90382_diag1
{
    uint16_t word_val;

    struct
    {
        uint8_t er_aroc     	: 1;    //BIT0
        uint8_t er_dsp   		: 1;
        uint8_t er_dsp_ta       : 1;
        uint8_t er_dsp_alf      : 1;
        uint8_t er_afe_ref      : 1;
        uint8_t er_adc_lin      : 1;
        uint8_t er_adc          : 1;
        uint8_t er_agc_lo       : 1;
        uint8_t er_agc_hi       : 1;
        uint8_t er_speed_hi     : 1;
        uint8_t er_temp_lo      : 1;
        uint8_t er_temp_hi      : 1;
        uint8_t er_nvm_sr       : 1;
        uint8_t er_ov_vdda      : 1;
        uint8_t er_ov_vddd      : 1;
        uint8_t er_ov_vaux      : 1;
    };
};

// ADDR = 0x030
union mlx90382_diag2
{
    uint16_t word_val;

    struct
    {
        uint8_t not_used0     	: 1;    //BIT0
        uint8_t er_nvm_ded   	: 1;
        uint8_t er_nvm_crc      : 1;
        uint8_t er_uv_vdd       : 1;
        uint8_t er_uv_vdda      : 1;
        uint8_t er_uv_vaux      : 1;
        uint8_t er_temp_max     : 1;
        uint8_t er_die       	: 1;
        uint8_t er_intp       	: 1;
        uint8_t er_scxy     	: 1;
        uint8_t not_used1      	: 6;
    };
};

// ADDR = 0x032
union mlx90382_diag3
{
    uint16_t word_val;

    struct
    {
        uint8_t diag_dsp     	: 1;    //BIT0
        uint8_t diag_agc_lo   	: 1;
        uint8_t diag_agc_hi   	: 1;
        uint8_t diag_dsp_ta     : 1;
        uint8_t diag_speed_hi   : 1;
        uint8_t diag_temp_lo    : 1;
        uint8_t diag_temp_hi    : 1;
        uint8_t diag_mem      	: 1;
        uint8_t diag_amp_adc    : 8;
    };
};

struct mlx90382_config {
	struct spi_dt_spec spi;
};

struct mlx90382_data {
	uint16_t lin_phase;
	uint16_t driftc_phase;
	uint16_t angle;
	uint16_t speed;
	uint16_t temp;
};

void cs_select(const struct gpio_dt_spec *cs)
{
    gpio_pin_set_dt(cs, 1);

	// k_msleep(1);
}

void cs_deselect(const struct gpio_dt_spec *cs)
{
	// k_msleep(1);

    gpio_pin_set_dt(cs, 0);
}

static const uint16_t crc_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

/**
 * Update the crc value with new data.
 *
 * \param crc      The current crc value.
 * \param data     Pointer to a buffer of \a data_len bytes.
 * \param data_len Number of bytes in the \a data buffer.
 * \return         The updated crc value.
 */
uint16_t crc_ccitt_update(uint16_t crc, const uint16_t* data, unsigned int data_len)
{
    unsigned int tbl_idx;

    while (data_len--) 
	{
        tbl_idx = ((crc >> 8) ^ (*data >> 8)) & 0xff;
        crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xffff;
        
		tbl_idx = ((crc >> 8) ^ (*data & 0xFF)) & 0xff;
        crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xffff;
        
		data++;
    }

	LOG_INF("crc_ccitt_update: crc = 0x%x\r\n", crc);
    
	return crc & 0xffff;
}

const uint8_t crc8x_table[] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

uint8_t frame_size = 0;

uint8_t crc8CCIT(uint8_t const *data, uint8_t dataSize)
{
    uint8_t crc = 0xff;

    for (uint8_t i = 0; i < dataSize; i++) 
	{
		LOG_INF("Data[%d]: 0x%02x", i, data[i]);

        crc = crc8x_table[crc ^ data[i]];
    }
    
	return crc ^ 0x00;
}

static int mlx90382_register_write(const struct device *dev, uint16_t reg_addr, uint16_t reg_val)
{
	int ret;
	
	const struct mlx90382_config *cfg = dev->config;
	
	uint8_t buffer_tx[5];
	uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
	
	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	buffer_tx[0] = (uint8_t)CMD_RW | (uint8_t)(reg_addr>>9);
	buffer_tx[1] = (reg_addr>>1) & 0xFF;
	buffer_tx[2] = (reg_val>>8) & 0xFF;
	buffer_tx[3] = reg_val & 0xFF;

#if 1	
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
#else
	/* 1. Lock the scheduler to prevent thread preemption */
    k_sched_lock();

    /* 2. Assert CS */
    gpio_pin_set_dt(&cs0, 1); 
	k_busy_wait(DELAY_US);

    /* 3. Perform the SPI transaction */
    /* Note: If this is a very long transfer, locking the scheduler 
       for the whole duration might impact system real-time performance. */
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

    /* 4. De-assert CS */
    gpio_pin_set_dt(&cs0, 0);
	k_busy_wait(DELAY_US);

    /* 5. Unlock the scheduler to allow context switching again */
    k_sched_unlock();	
#endif

	if (ret != 0) {
		LOG_ERR("%s: spi_transceive failed with error %i", dev->name, ret);
		return ret;
	}

	LOG_INF("mlx90382_register_write: reg_addr=0x%04x, reg_val=0x%04x", reg_addr, reg_val);
	LOG_INF("tx: 0x%x 0x%x 0x%x 0x%x", buffer_tx[0], buffer_tx[1], buffer_tx[2], buffer_tx[3]);
	LOG_INF("rx: 0x%x 0x%x 0x%x 0x%x 0x%x", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3], buffer_rx[4]);

	return ret;
}

static int mlx90382_register_read(const struct device *dev, uint16_t reg_addr, uint8_t *buffer, size_t reg_size)
{
	int ret;
	
	const struct mlx90382_config *cfg = dev->config;
	
	uint8_t buffer_tx[5];
	// uint8_t buffer_rx[ARRAY_SIZE(buffer_tx)];
	uint8_t buffer_rx[reg_size+3];
	
	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	// if (reg_addr >= MLX90382_NVRAM_BASE_ADDR)
	// {
	// 	//Stop Application mode to access NVRAM
	// 	ret = mlx90382_register_write(dev, MLX90382_APPLICATION_MODE, 2);
	// 	if (ret != 0) {
	// 		LOG_ERR("%s: Failed to stop application mode with error %i", dev->name, ret);
	// 		return ret;
	// 	}
	// }

	buffer_tx[0] = (uint8_t)CMD_RR | (reg_addr>>9);
	buffer_tx[1] = (reg_addr>>1) & 0xFF;
	buffer_tx[2] = 0x00;
	buffer_tx[3] = 0x00;

#if 1
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
#else
	/* 1. Lock the scheduler to prevent thread preemption */
    k_sched_lock();

    /* 2. Assert CS */
    gpio_pin_set_dt(&cs0, 1); 
	k_busy_wait(DELAY_US);

    /* 3. Perform the SPI transaction */
    /* Note: If this is a very long transfer, locking the scheduler 
       for the whole duration might impact system real-time performance. */
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

    /* 4. De-assert CS */
    gpio_pin_set_dt(&cs0, 0);
	k_busy_wait(DELAY_US);
	
    /* 5. Unlock the scheduler to allow context switching again */
    k_sched_unlock();	
#endif

	if (ret != 0) {
		LOG_ERR("%s: spi_transceive failed with error %i", dev->name, ret);
		return ret;
	}

	for (uint8_t i = 0; i < reg_size; i++) {
		buffer[i] = buffer_rx[3 + i];
	}

	// LOG_INF("tx: 0x%x 0x%x 0x%x 0x%x", buffer_tx[0], buffer_tx[1], buffer_tx[2], buffer_tx[3]);
	// LOG_INF("rx: 0x%x 0x%x 0x%x 0x%x 0x%x", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3], buffer_rx[4]);

	// if (reg_addr >= MLX90382_NVRAM_BASE_ADDR)
	// {
	// 	//Active Application mode to start DSP calculation
	// 	ret = mlx90382_register_write(dev, MLX90382_APPLICATION_MODE, 0);
	// 	if (ret != 0) {
	// 		LOG_ERR("%s: Failed to active application mode with error %i", dev->name, ret);
	// 		return ret;
	// 	}
	// }

	return ret;
}

/* Using FR mode needs a configuration of GPIO_IF[1:0] = 2 (SPI-Bus mode) */
static int mlx90382_frame_read(const struct device *dev, uint8_t *buff, size_t buff_size)
{
	int ret;
    uint8_t crc8;
	
	const struct mlx90382_config *cfg = dev->config;
	
	uint8_t buffer_tx[buff_size+2];
	uint8_t buffer_rx[buff_size+2];
	
	const struct spi_buf tx_buf[] = {{
		.buf = buffer_tx,
		.len = ARRAY_SIZE(buffer_tx),
	}};
	
	const struct spi_buf rx_buf[] = {{
		.buf = buffer_rx,
		.len = ARRAY_SIZE(buffer_rx),
	}};
	
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	buffer_tx[0] = (uint8_t)CMD_FR;
	buffer_tx[1] = 0x00;

#if 1
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
#else
	/* 1. Lock the scheduler to prevent thread preemption */
    k_sched_lock();

    /* 2. Assert CS */
    gpio_pin_set_dt(&cs0, 1); 
	k_busy_wait(DELAY_US);

    /* 3. Perform the SPI transaction */
    /* Note: If this is a very long transfer, locking the scheduler 
       for the whole duration might impact system real-time performance. */
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

    /* 4. De-assert CS */
    gpio_pin_set_dt(&cs0, 0);
	k_busy_wait(DELAY_US);
	
    /* 5. Unlock the scheduler to allow context switching again */
    k_sched_unlock();	
#endif

	if (ret != 0) {
		LOG_ERR("%s: spi_transceive failed with error %i", dev->name, ret);
		return ret;
	}

	for (uint8_t i = 0; i < buff_size; i++) {
		buff[i] = buffer_rx[2 + i];
	}

	// LOG_INF("spi: 0x%x config: 0x%x 0x%x 0x%x 0x%x 0x%x", cfg->spi, cfg->spi.config.frequency, cfg->spi.config.operation, cfg->spi.config.slave, cfg->spi.config.cs.gpio, cfg->spi.config.cs);
	
	// LOG_INF("tx: 0x%x 0x%x 0x%x 0x%x\r\n", buffer_tx[0], buffer_tx[1], buffer_tx[2], buffer_tx[3]);
	// LOG_INF("rx: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3], buffer_rx[4], buffer_rx[5], buffer_rx[6], buffer_rx[7]);

	crc8 = crc8CCIT(buff, buff_size);
	if (crc8 != 0) 
	{
		LOG_ERR("%s: Frame CRC check failed, crc8 = 0x%02x\r\n", dev->name, crc8);
	
		return -1;
	}

	return ret;
}

int mlx90382_nvram_read(const struct device *dev, uint16_t nvram_addr, uint8_t *buffer, size_t nvram_size)
{
	int res = 0;

	//Stop Application mode to access NVRAM
	res = mlx90382_register_write(dev, 0x024, 2);

	res = mlx90382_shadow_register_monitor_onoff(dev, 0);

    mlx90382_register_read(dev, nvram_addr, buffer, nvram_size);

	res = mlx90382_soft_reset(dev);

	return res;
}

int mlx90382_shadow_register_read(const struct device *dev, uint16_t nvram_addr, uint8_t *buffer, size_t nvram_size)
{
	int res = 0;

	//Stop Application mode to access NVRAM
	// res = mlx90382_register_write(dev, 0x024, 2);

    mlx90382_register_read(dev, nvram_addr, buffer, nvram_size);

	//Set Application mode to active
	// res = mlx90382_register_write(dev, 0x024, 0);

	return res;
}

int mlx90382_frame_size(const struct device *dev)
{
	int frfsen  = 0;
	int frcrcen = 0;
	int faddr1  = 0;
	int faddr2  = 0;
	int faddr3  = 0;

	int frame_size = 0;

	uint8_t buf[2];

	union mlx90382_spi_config spi_config;
	union mlx90382_spi_faddr01 spi_faddr01;
	union mlx90382_spi_faddr23 spi_faddr23;

	mlx90382_register_read(dev, MLX90382_NV_SPI_CONFIG, buf, 2);
	spi_config.word_val = buf[0]<<8 | buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR01, buf, 2);
	spi_faddr01.word_val = buf[0]<<8 | buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR23, buf, 2);
	spi_faddr23.word_val = buf[0]<<8 | buf[1];
	
	frfsen = spi_config.spi_frfsen;
	frcrcen = spi_config.spi_frcrcen;
	faddr1 = spi_faddr01.spi_faddr1;
	faddr2 = spi_faddr23.spi_faddr2;
	faddr3 = spi_faddr23.spi_faddr3;

	frame_size = (frfsen ? 1 : 0) + (frcrcen ? 1 : 0) + 2*(1 + (faddr1>0) + (faddr2>0) + (faddr3>0));
	LOG_INF("frfsen = 0x%x, frcrcen = 0x%x, spi_config = 0x%x, faddr1 = 0x%x, faddr2 = 0x%x, faddr3 = 0x%x\r\n", frfsen, frcrcen, spi_config.word_val, faddr1, faddr2, faddr3);
	LOG_INF("frame size is %d bytes\r\n", frame_size);

	return frame_size;
}

int mlx90382_get_crc(const struct device *dev, uint16_t *crc)
{
	int rec = 0;
	uint8_t buf[2];
	union mlx90382_crc_calc_reg crc_calc_reg;

	// mlx90382_register_read(dev, MLX90382_CRC_RESULT, buf, 2);
	// *crc = buf[0]<<8 || buf[1];
	// LOG_ERR("MLX90382_CRC_RESULT = 0x%x\r\n", *crc);	

	// crc_calc_reg.word_val = 0x0000;
	// crc_calc_reg.crc_calc_strt = 0x0001;
	mlx90382_register_write(dev, MLX90382_CRC_CALC, 0x0000);
	mlx90382_register_write(dev, MLX90382_CRC_CALC, 0x0001);

	k_msleep(100);

	do 
	{
		mlx90382_register_read(dev, MLX90382_CRC_CALC, buf, 2);
		crc_calc_reg.word_val = (buf[0]<<8) + buf[1];

		LOG_INF("crc_calc_reg = 0x%x\r\n", crc_calc_reg.word_val);
	}
	while (crc_calc_reg.crc_calc_done == 0x0000);
	// while (crc_calc_reg.word_val == 0x0000);

	// while (mlx90382_register_read(dev, MLX90382_CRC_CALC, (uint8_t *)&crc_calc_reg, 2) == 0)
	// {
	// 	// if (crc_calc_reg.crc_calc_done == 0x0001)
	// 	if (crc_calc_reg.word_val != 0x000)
	// 	{
	// 		LOG_INF("crc_calc_reg = 0x%x\r\n", crc_calc_reg.word_val);

	// 		break;
	// 	}

	// 	LOG_INF("claculating crc... crc_cal_reg = 0x%x\r\n", crc_calc_reg.word_val);
	// }

	mlx90382_register_read(dev, MLX90382_CRC_RESULT, buf, 2);
	*crc = (buf[0]<<8) + buf[1];
	LOG_ERR("MLX90382_CRC_RESULT = 0x%x\r\n", *crc);

	return rec;
}

int mlx90382_set_crc(const struct device *dev, uint16_t crc)
{
	int rec = 0;

	LOG_ERR("set crc in nvram[0x%x]: 0x%x\r\n", MLX90382_NV_CUS_CRC_ADDR, crc);
	rec = mlx90382_register_write(dev, MLX90382_NV_CUS_CRC_ADDR, crc);

	return rec;
}

int mlx90382_nvram_store(const struct device *dev)
{
	int rec = 0;

	uint16_t Key1 = 0xb;
    uint16_t Key2 = 0x6;
    uint16_t Key3 = (uint16_t)(0x1 << 0x8);

	rec = mlx90382_register_write(dev, NVRAM_NVOP_KEY_ADDRESS, Key1);
	rec = mlx90382_register_write(dev, NVRAM_NVOP_KEY_ADDRESS, Key2);
	rec = mlx90382_register_write(dev, NVRAM_STORE_REQ_ADDRESS, Key3);

	return rec;
}

int mlx90382_read_nvram(const struct device *dev, uint16_t *data_rx, uint16_t n)
{
	int res = 0;

	uint8_t buf[2];

	// mlx90382_set_application_mode(dev, 2);

#if 0	
	for (uint16_t i = 0; i < n; i++)
	{
		res = mlx90382_register_read(dev, MLX90382_NVRAM_BASE_ADDR + i*2, buf, 2);
		
		data_rx[i] = (buf[0]<<8) + buf[1];
		// data_rx[i+1] = buf[1];

		LOG_INF("NVRAM[0x%04x] = 0x%04x\r\n", MLX90382_NVRAM_BASE_ADDR + i*2, data_rx[i]);
	}
#else
	res = mlx90382_nvram_read(dev, MLX90382_NVRAM_BASE_ADDR, (uint8_t *)data_rx, n * 2);

	for (uint16_t i = 0; i < n; i++)
	{
		buf[0] = data_rx[i] & 0xFF;
		buf[1] = (data_rx[i] >> 8) & 0xFF;

		data_rx[i] = (buf[0]<<8) + buf[1];

		if ((i > n-2) || (i < 5))
			LOG_ERR("NVRAM[0x%04x] = 0x%04x", MLX90382_NVRAM_BASE_ADDR + i*2, data_rx[i]);
	}	
#endif

	// mlx90382_soft_reset(dev);

    if (crc_ccitt_update(0xffff, data_rx, NVRAM_SIZE_WORDS) != 0) 
	{
		LOG_ERR("CRC check failed for NVRAM data\r\n");
		return -1;
    } 
	else 
	{
		LOG_INF("CRC check passed for NVRAM data\r\n");
    }	
}

int mlx90382_nvram_crc_update(const struct device *dev)
{
	int res = 0;
    uint16_t crc;
	uint8_t buf[2];
    uint16_t val;

	//Disable Shadow Register Monitor
	res = mlx90382_register_write(dev, 0x0BE, 3<<1);
	mlx90382_register_read(dev, 0x0BE, buf, 2);
	val = (buf[0]<<8) + buf[1];
	if (val != 6)
	{
		LOG_ERR("Disable Shadow Register Monitor failed, %x", val);
	}

	//Stop Application mode to access NVRAM
	//Stop automatic NVRAM CRC calculation
	res = mlx90382_register_write(dev, 0x024, 6);
	mlx90382_register_read(dev, 0x024, buf, 2);
	val = (buf[0]<<8) + buf[1];
	if (val != 6)
	{
		LOG_ERR("Stop Application mode to access NVRAM failed, %x", val);
	}

	res = mlx90382_get_crc(dev, &crc);
	res = mlx90382_set_crc(dev, crc);

	res = mlx90382_nvram_store(dev);

	k_msleep(1000);
	
	mlx90382_soft_reset(dev);
}

int mlx90382_program_nvram(const struct device *dev, uint16_t addr, uint16_t val)
{
	int res = 0;
    uint16_t crc;
    uint8_t buffer[2];

	//Disable Shadow Register Monitor
	res = mlx90382_register_write(dev, 0x0BE, 3<<1);

	//Stop Application mode to access NVRAM
	//Stop automatic NVRAM CRC calculation
	res = mlx90382_register_write(dev, 0x024, 6);

	res = mlx90382_register_write(dev, addr, val);

	res = mlx90382_get_crc(dev, &crc);
	res = mlx90382_set_crc(dev, crc);

	res = mlx90382_nvram_store(dev);

	k_msleep(1000);
	
	mlx90382_soft_reset(dev);

	return res;
}

int mlx90382_set_sensing_mode(const struct device *dev, uint16_t mode)
{
    int res = 0;
    uint8_t buf[2];

    union mlx90382_config_reg reg;

    mlx90382_register_read(dev, MLX90382_NV_CONFIG_REG, buf, 2);

    reg.word_val = (buf[0]<<8) + buf[1];

    reg.sensing_mode = mode;

	res = mlx90382_program_nvram(dev, MLX90382_NV_CONFIG_REG, reg.word_val);

    return res;
}

int mlx90382_set_application_mode(const struct device *dev, uint16_t onoff)
{
	int res = 0;

	#if 0
	uint8_t buf[2];

	union mlx90382_application_mode_reg reg;

	mlx90382_register_read(dev, MLX90382_APPLICATION_MODE, buf, 2);

	reg.word_val = (buf[0]<<8) + buf[1];

	reg.application_mode = onoff;

	res = mlx90382_register_write(dev, MLX90382_APPLICATION_MODE, reg.word_val);
#else
	res = mlx90382_register_write(dev, MLX90382_APPLICATION_MODE, 2);
#endif

	return res;
}

int mlx90382_get_config(const struct device *dev, union mlx90382_config_reg *config)
{
    int res = 0;
    uint8_t buf[2];

#if 0	
    res = mlx90382_nvram_read(dev, MLX90382_NV_CONFIG_REG, buf, 2);
    if (res != 0)
    {
		LOG_ERR("mlx90382_get_config is failed\r\n");

        return res;
    }
#else
    res = mlx90382_register_read(dev, MLX90382_NV_CONFIG_REG, buf, 2);
    if (res != 0)
    {
		LOG_ERR("mlx90382_get_config is failed\r\n");

        return res;
    }
#endif

    config->word_val = ((buf[0] << 8) + buf[1]);
	// LOG_INF("mlx90382_get_config: 0x%04x\r\n", config->word_val);
	
    return 0;
}

int mlx90382_get_analog_version(const struct device *dev, uint16_t *version)
{
    int res = 0;
    uint8_t buf[2];

    res = mlx90382_register_read(dev, MLX90382_ANA_VERSION, buf, 2);
    if (res != 0)
    {
        return res;
    }

    *version = (buf[0]<<8) + buf[1];
    // LOG_INF("mlx90382_get_analog_version: 0x%04x\r\n", *version);

    return 0;
}

int mlx90382_get_digital_version(const struct device *dev, uint16_t *version)
{
    int res = 0;
    uint8_t buf[2];

    res = mlx90382_register_read(dev, MLX90382_DIG_VERSION, buf, 2);
    if (res != 0)
    {
        return res;
    }

    *version = (buf[0]<<8) + buf[1];
	// LOG_INF("mlx90382_get_digital_version: 0x%04x\r\n", *version);

    return 0;
}

int mlx90382_get_zero_position(const struct device *dev, uint16_t *position)
{
    int res = 0;
    uint8_t buf[2];

#if 0	
    res = mlx90382_nvram_read(dev, MLX90382_NV_PHASE_OFS, buf, 2);
    if (res != 0)
    {
        LOG_ERR("mlx90382_get_zero_position is failed\r\n");
        return res;
    }
#else
    res = mlx90382_register_read(dev, MLX90382_NV_PHASE_OFS, buf, 2);
    if (res != 0)
    {
        LOG_ERR("mlx90382_get_zero_position is failed\r\n");
        return res;
    }
#endif

    *position = (buf[0]<<8) + buf[1];
    // LOG_INF("mlx90382_get_zero_position: 0x%04x\r\n", *position);

    return 0;
}

int mlx90382_get_info(const struct device *dev)
{
    int result = 0;

	uint8_t buf[2];

    uint16_t dev_info;
    uint16_t a_ver;
    uint16_t d_ver_h;
    uint32_t d_ver;
    uint16_t zero_position;
    uint16_t sc_x1;
    uint16_t sc_x2;
    uint16_t sc_y1;
    uint16_t sc_y2;
    uint16_t sc_ye;
    uint16_t sc_hl;

	uint8_t faddr0;
	uint8_t faddr1;
	uint8_t faddr2;
	uint8_t faddr3;

	union mlx90382_spi_config spi_config;
	union mlx90382_state state;
	union mlx90382_diag1 diag1;
	union mlx90382_diag2 diag2;
	union mlx90382_diag3 diag3;
	union mlx90382_0xbe val_0xbe;
    union mlx90382_config_reg config;

	mlx90382_register_read(dev, MLX90382_DEV_INFO, buf, 2);
	dev_info = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, 0xBE, buf, 2);
	val_0xbe.word_val = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_STATE, buf, 2);
	state.word_val = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_DIAG1, buf, 2);
	diag1.word_val = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_DIAG2, buf, 2);
	diag2.word_val = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_DIAG3, buf, 2);
	diag3.word_val = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SC_X1, buf, 2);
	sc_x1 = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SC_X2, buf, 2);
	sc_x2 = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SC_Y1, buf, 2);
	sc_y1 = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SC_Y2, buf, 2);
	sc_y2 = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SC_YE, buf, 2);
	sc_ye = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SC_HL, buf, 2);
	sc_hl = (buf[0]<<8) + buf[1];

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR01, buf, 2);
	faddr0 = buf[1];	
	faddr1 = buf[0];	

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR23, buf, 2);
	faddr2 = buf[1];	
	faddr3 = buf[0];	

	mlx90382_register_read(dev, MLX90382_NV_SPI_CONFIG, buf, 2);
	spi_config.word_val = (buf[0] << 8) + buf[1];	

    mlx90382_get_config(dev, &config);
    mlx90382_get_zero_position(dev, &zero_position);
    mlx90382_get_digital_version(dev, &d_ver_h);
    mlx90382_get_analog_version(dev, &a_ver);
    d_ver = (a_ver>>8) + (d_ver_h<<16);

    printf("dev_info:%x\n", dev_info);
    printf("aversion:%x\n", a_ver&0xFF);
    printf("dversion:%x\n", d_ver);
    printf("zeropos:0x%x\n", zero_position);
    printf("sc_x1:0x%x\n", sc_x1);
    printf("sc_x2:0x%x\n", sc_x2);
    printf("sc_y1:0x%x\n", sc_y1);
    printf("sc_y2:0x%x\n", sc_y2);
    printf("sc_ye:0x%x\n", sc_ye);
    printf("sc_hl:0x%x\n", sc_hl);
    printf("config:0x%x\n", config.word_val);
	printf("ram_0xbe:0x%x\n", val_0xbe.word_val);
	printf("state:0x%x\n", state.state);
	printf("diag1:0x%x\n", diag1.word_val);
	printf("diag2:0x%x\n", diag2.word_val);
	printf("diag3:0x%x\n", diag3.word_val);
	printf("faddr0:0x%x\n", faddr0);
	printf("faddr1:0x%x\n", faddr1);
	printf("faddr2:0x%x\n", faddr2);
	printf("faddr3:0x%x\n", faddr3);
	printf("spi_conf:0x%x\n", spi_config.word_val);

    return result;
}

int mlx90382_soft_reset(const struct device *dev)
{
    int res = 0;

    res = mlx90382_register_write(dev, MLX90382_SOFT_RESET, 2);

    return res;
}

int mlx90382_set_zero_position(const struct device *dev, uint16_t position)
{
    int res = 0;

	res = mlx90382_program_nvram(dev, MLX90382_NV_PHASE_OFS, position);
	// res = mlx90382_register_write(dev, MLX90382_NV_PHASE_OFS, position);

    if (res != 0)
    {
        LOG_INF("mlx90382_set_zero_position is failed\r\n");
    }

    return res;
}

int mlx90382_set_gpio_if(const struct device *dev, uint8_t val)
{
    int ret = 0;
    uint8_t buf[4];

    union mlx90382_config_reg reg;

#if 1
	mlx90382_register_read(dev, MLX90382_NV_CONFIG_REG, buf, 2);
	
    reg.word_val = (buf[0]<<8) + buf[1];

    reg.gpio_if = val;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_CONFIG_REG, reg.word_val);
#else
	mlx90382_register_read(dev, MLX90382_NV_CONFIG_REG, buf, 2);
	
    reg.word_val = (buf[0]<<8) + buf[1];

    reg.gpio_if = val;

	ret = mlx90382_register_write(dev, MLX90382_NV_CONFIG_REG, reg.word_val);
#endif

    return ret;
}

int mlx90382_set_faddr0(const struct device *dev, uint8_t addr)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_faddr01 faddr01;

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR01, buf, 2);
	
    faddr01.word_val = (buf[0]<<8) + buf[1];

    faddr01.spi_faddr0 = addr;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_FADDR01, faddr01.word_val);

    return ret;
}

int mlx90382_set_faddr1(const struct device *dev, uint8_t addr)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_faddr01 faddr01;

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR01, buf, 2);
	
    faddr01.word_val = (buf[0]<<8) + buf[1];

    faddr01.spi_faddr1 = addr;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_FADDR01, faddr01.word_val);

    return ret;
}

int mlx90382_set_faddr2(const struct device *dev, uint8_t addr)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_faddr23 faddr23;

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR23, buf, 2);
	
    faddr23.word_val = (buf[0]<<8) + buf[1];

    faddr23.spi_faddr2 = addr;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_FADDR23, faddr23.word_val);

    return ret;
}

int mlx90382_set_faddr3(const struct device *dev, uint8_t addr)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_faddr23 faddr23;

	mlx90382_register_read(dev, MLX90382_NV_SPI_FADDR23, buf, 2);
	
    faddr23.word_val = (buf[0]<<8) + buf[1];

    faddr23.spi_faddr3 = addr;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_FADDR23, faddr23.word_val);

    return ret;
}

int mlx90382_set_frfs(const struct device *dev, uint8_t frfs)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_config cfg;

	mlx90382_register_read(dev, MLX90382_NV_SPI_CONFIG, buf, 2);
	
    cfg.word_val = (buf[0]<<8) + buf[1];

    cfg.spi_frfs = frfs;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_CONFIG, cfg.word_val);

    return ret;
}

int mlx90382_set_frfsen(const struct device *dev, uint8_t onoff)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_config cfg;

	mlx90382_register_read(dev, MLX90382_NV_SPI_CONFIG, buf, 2);
	
    cfg.word_val = (buf[0]<<8) + buf[1];

    cfg.spi_frfsen = onoff;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_CONFIG, cfg.word_val);

    return ret;
}

int mlx90382_set_frcrcen(const struct device *dev, uint8_t onoff)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_config cfg;

	mlx90382_register_read(dev, MLX90382_NV_SPI_CONFIG, buf, 2);
	
    cfg.word_val = (buf[0]<<8) + buf[1];

    cfg.spi_frcrcen = onoff;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_CONFIG, cfg.word_val);

    return ret;
}

int mlx90382_set_frinv(const struct device *dev, uint8_t inv)
{
    int ret = 0;
    uint8_t buf[2];

	union mlx90382_spi_config cfg;

	mlx90382_register_read(dev, MLX90382_NV_SPI_CONFIG, buf, 2);
	
    cfg.word_val = (buf[0]<<8) + buf[1];

    cfg.spi_frinv = inv;

	ret = mlx90382_program_nvram(dev, MLX90382_NV_SPI_CONFIG, cfg.word_val);

    return ret;
}

/* API implementation */
static int mlx90382_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val)
{
	// const struct npm10xx_adc_config *config = dev->config;

	// if (attr != SENSOR_ATTR_FULL_SCALE ||
	//     (chan != SENSOR_CHAN_CURRENT && chan != SENSOR_CHAN_GAUGE_AVG_CURRENT)) {
	// 	LOG_ERR("Only current full scale attribute is supported");
	// 	return -ENOTSUP;
	// }

	// ARRAY_FOR_EACH(ibat_fullscale_ua, idx) {
	// 	if (sensor_value_to_micro(val) == ibat_fullscale_ua[idx]) {
	// 		return i2c_reg_write_byte_dt(&config->i2c, NPM10_CHARGER_IBATFS,
	// 					     FIELD_PREP(CHARGER_IBATFS_LVL_Msk, idx));
	// 	}
	// }

	return -EINVAL;
}

static int mlx90382_attr_get(const struct device *dev, enum sensor_channel chan,
				   enum sensor_attribute attr, struct sensor_value *val)
{
	const struct mlx90382_config *config = dev->config;
	struct mlx90382_data *data = dev->data;

	int ret;
	uint8_t reg;

	// if (attr != SENSOR_ATTR_FULL_SCALE ||
	//     (chan != SENSOR_CHAN_CURRENT && chan != SENSOR_CHAN_GAUGE_AVG_CURRENT)) {
	// 	LOG_ERR("Only current full scale attribute is supported");
	// 	return -ENOTSUP;
	// }

	// ret = i2c_reg_read_byte_dt(&config->i2c, NPM10_CHARGER_IBATFS, &reg);
	// if (ret < 0) {
	// 	return ret;
	// }

	// return sensor_value_from_micro(val,
	// 			       ibat_fullscale_ua[FIELD_GET(CHARGER_IBATFS_LVL_Msk, reg)]);
}

static int mlx90382_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mlx90382_data *data = dev->data;
	const struct mlx90382_config *cfg = dev->config;

	uint8_t buffer[9];

	int ret;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	switch (chan) {
		case SENSOR_CHAN_LIN_PHASE:
			ret = mlx90382_register_read(dev, MLX90382_LIN_PHASE, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->lin_phase = (buffer[0] << 8) | buffer[1];
			break;

		case SENSOR_CHAN_DRIFTC_PHASE:
			ret = mlx90382_register_read(dev, MLX90382_DRIFTC_PHASE, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->driftc_phase = (buffer[0] << 8) | buffer[1];
			break;			

		case SENSOR_CHAN_SC_PHASE:
			ret = mlx90382_register_read(dev, MLX90382_SC_PHASE, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->angle = (buffer[0] << 8) | buffer[1];
			break;

		case SENSOR_CHAN_DIE_TEMP:
			ret = mlx90382_register_read(dev, MLX90382_TEMP, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->temp = (buffer[0] << 8) | buffer[1];
			break;

		case SENSOR_CHAN_RPM:
			ret = mlx90382_register_read(dev, MLX90382_SPEED, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->speed = (buffer[0] << 8) | buffer[1];
			break;
			
		case SENSOR_CHAN_ALL:
#if 1		
			ret = mlx90382_register_read(dev, MLX90382_LIN_PHASE, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->lin_phase = (buffer[0] << 8) | buffer[1];

			ret = mlx90382_register_read(dev, MLX90382_DRIFTC_PHASE, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->driftc_phase = (buffer[0] << 8) | buffer[1];			

			ret = mlx90382_register_read(dev, MLX90382_SC_PHASE, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->angle = (buffer[0] << 8) | buffer[1];

			ret = mlx90382_register_read(dev, MLX90382_TEMP, buffer, 2);
			if (ret < 0) {
				LOG_ERR("spi transceive failed (%d)", ret);
				return ret;
			}

			data->temp = (buffer[0] << 8) | buffer[1];
#else		
			mlx90382_frame_read(dev, buffer, frame_size);

			data->lin_phase 	= (buffer[1] << 8) | buffer[2];
			data->driftc_phase 	= (buffer[3] << 8) | buffer[4];			
			data->angle 		= (buffer[5] << 8) | buffer[6];
			data->temp 			= (buffer[7] << 8) | buffer[8];
#endif			
			break;
	
		default:
			return -ENOTSUP;
	}

	return 0;
}

static int mlx90382_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct mlx90382_data *data = dev->data;
	float lin_phase;
	float driftc_phase;
	float angle_deg;
	float temp_celsius;
	float speed_rpm;

	switch (chan) {
		case SENSOR_CHAN_LIN_PHASE:
			lin_phase = (float)data->lin_phase;
			lin_phase = (lin_phase / 0x10000) * MLX90382_FULL_ANGLE_DEG;
			sensor_value_from_float(val, lin_phase);
			break;

		case SENSOR_CHAN_DRIFTC_PHASE:
			driftc_phase = (float)data->driftc_phase;
			driftc_phase = (driftc_phase / 0x10000) * MLX90382_FULL_ANGLE_DEG;
			sensor_value_from_float(val, driftc_phase);
			break;			

		case SENSOR_CHAN_SC_PHASE:
			angle_deg = (float)data->angle;
			angle_deg = (angle_deg / MLX90382_MAX_STEPS) * MLX90382_FULL_ANGLE_DEG * 2;
			sensor_value_from_float(val, angle_deg);

			// printf("0x%04x[%d] angle: %f deg\r\n", data->angle, data->angle, angle_deg);
			break;

		case SENSOR_CHAN_DIE_TEMP:
			temp_celsius = (float)(data->temp);
			temp_celsius = temp_celsius / 8 + 200 - 273.15;
			sensor_value_from_float(val, temp_celsius);
			break;
		
		case SENSOR_CHAN_RPM:
			speed_rpm = (float)(data->speed);
			sensor_value_from_float(val, speed_rpm);
			break;

		default:
			return -ENOTSUP;
	}

#if 0	
	int32_t scaled = (int32_t)data->angle_raw * MLX90382_FULL_ANGLE_DEG;

	val->val1 = scaled / MLX90382_MAX_STEPS;
	val->val2 = ((scaled % MLX90382_MAX_STEPS) * MLX90382_MICRO_DEGREE) / MLX90382_MAX_STEPS;
#else
	// val->val1 = data->angle_raw;
	// val->val2 = 0;
#endif

	return 0;
}

void mlx90382_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	int rc;
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	struct mlx90382_data *data = dev->data;
	uint64_t cycles;

	printf("mlx90382_submit\r\n");

#if 0	
	rc = mlx90382_trigger_measurement_internal(dev, cfg->channels->chan_type);
	if (rc != 0) {
		LOG_ERR("Failed to trigger measurement");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		LOG_ERR("Failed to get sensor clock cycles");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	/* save information for the work item */
	data->work_ctx.timestamp = sensor_clock_cycles_to_ns(cycles);
	data->work_ctx.iodev_sqe = iodev_sqe;
	data->work_ctx.config_val = data->config_val;
#endif

	/* schedule work to read out sensor and inform the executor about completion with success */
	// k_work_schedule(&data->async_fetch_work, K_USEC(data->measurement_time_us));
}

/*
Note that if one writes parameters in the SR, Shadow Register, DE_SR
needs to remain disabled. If not, the changes in the SR register will be overwritten with the NVRAM content
by the sensor.
 */
int mlx90382_shadow_register_monitor_onoff(const struct device *dev, uint8_t onoff)
{
	uint8_t buffer[2];
	union mlx90382_0xbe val_0xbe;

	mlx90382_register_read(dev, 0xBE, buffer, 2);
	val_0xbe.word_val = (buffer[0]<<8) + buffer[1];
	LOG_INF("Current [0xBE] = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);	

	if (onoff)
	{
		val_0xbe.de_sr = 0;
	}
	else
	{
		val_0xbe.de_sr = 3;
	}

	LOG_INF("[0xBE] = 0x%x\r\n", val_0xbe.word_val);
	mlx90382_register_write(dev, 0xBE, val_0xbe.word_val);

	return 0;
}

static int mlx90382_init(const struct device *dev)
{
	int ret;
	uint8_t buffer[4];
	uint8_t frame_buffer[6];

    uint16_t nvram_data[NVRAM_SIZE_WORDS];
	union mlx90382_config_reg config_reg;
	union mlx90382_diag2 diag2_reg;

	const struct mlx90382_config *cfg = dev->config;

	LOG_INF("mlx90382_init");

#if 0	
	if (!gpio_is_ready_dt(&cs0)) {
		LOG_ERR("gpio device not ready");
		LOG_INF("gpio device not ready");

		return -1;
	}

	// ret = gpio_pin_configure_dt(&cs0, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_LOW);
	ret = gpio_pin_configure_dt(&cs0, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("gpio pin configure failed");
		LOG_INF("gpio pin configure failed");
		
		return -1;
	}
#endif

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	mlx90382_register_read(dev, MLX90382_DEV_INFO, buffer, 2);
	LOG_INF("device info: 0x%04x", (buffer[0]<<8) + buffer[1]);

	mlx90382_register_read(dev, MLX90382_DIAG2, buffer, 2);
	diag2_reg.word_val = (buffer[0]<<8) + buffer[1];
	if (diag2_reg.er_nvm_crc == 1)
	{
		LOG_ERR("ER_NVRAM_CRC![0x%x]", diag2_reg.word_val);
		mlx90382_nvram_crc_update(dev);
	}

	// mlx90382_register_read(dev, MLX90382_STATE, buffer, 2);
	// if (buffer[0] != 0 || buffer[1] != 0)
	// {
	// 	LOG_WRN("STATE = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);
	// }

	// mlx90382_register_read(dev, MLX90382_DIAG1, buffer, 2);
	// if (buffer[0] != 0 || buffer[1] != 0)
	// {
	// 	LOG_WRN("DIAG1 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);	
	// }

	// mlx90382_register_read(dev, MLX90382_DIAG2, buffer, 2);
	// if (buffer[0] != 0 || buffer[1] != 0)
	// {
	// 	LOG_WRN("DIAG2 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);	
	// }

	// mlx90382_register_read(dev, MLX90382_DIAG3, buffer, 2);
	// if (buffer[0] != 0 || buffer[1] != 0)
	// {
	// 	LOG_WRN("DIAG3 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		
	// }

	// mlx90382_register_read(dev, MLX90382_NV_SC_X1, buffer, 2);
	// LOG_WRN("SC_X1 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		

	// mlx90382_register_read(dev, MLX90382_NV_SC_X2, buffer, 2);
	// LOG_WRN("SC_X2 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		

	// mlx90382_register_read(dev, MLX90382_NV_SC_Y1, buffer, 2);
	// LOG_WRN("SC_Y1 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		

	// mlx90382_register_read(dev, MLX90382_NV_SC_Y2, buffer, 2);
	// LOG_WRN("SC_Y2 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		
	
#if 0	
	mlx90382_shadow_register_monitor_onoff(dev, 0);

	// mlx90382_nvram_read(dev, MLX90382_NV_CONFIG_REG, buffer, 2);
	mlx90382_register_read(dev, MLX90382_NV_CONFIG_REG, buffer, 2);
	config_reg.word_val = (buffer[0]<<8) + buffer[1];
	LOG_INF("config reg = 0x%x\r\n", config_reg.word_val);
	
	// disable NVRAM CRC calculation and set Application mode inactive
	// mlx90382_register_write(dev, 0x024, 6);
	
	config_reg.gpio_if = 2;	//SPI bus mode
	mlx90382_register_write(dev, MLX90382_NV_CONFIG_REG, config_reg.word_val);
	
	// mlx90382_nvram_read(dev, MLX90382_NV_CONFIG_REG, buffer, 2);
	mlx90382_register_read(dev, MLX90382_NV_CONFIG_REG, buffer, 2);
	config_reg.word_val = (buffer[0]<<8) + buffer[1];
	LOG_INF("config reg = 0x%x\r\n", config_reg.word_val);
#endif

#if 1	
	// mlx90382_register_write(dev, MLX90382_APPLICATION_MODE, 0);

	// mlx90382_register_read(dev, MLX90382_APPLICATION_MODE, buffer, 2);
	// LOG_INF("application mode: 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);

	// mlx90382_register_read(dev, 0xBE, buffer, 2);
	// LOG_INF("[0xBE] = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);	

	// mlx90382_register_read(dev, MLX90382_DIAG1, buffer, 2);
	// LOG_INF("DIAG1 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		

	// mlx90382_register_read(dev, MLX90382_DIAG2, buffer, 2);
	// LOG_INF("DIAG2 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		

	// mlx90382_register_read(dev, MLX90382_DIAG3, buffer, 2);
	// LOG_INF("DIAG3 = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);		

	// mlx90382_register_read(dev, MLX90382_TEMP, buffer, 2);
	// mlx90382_register_read(dev, MLX90382_SC_PHASE, buffer, 2);

    // set GPIO_IF as SPI bus mode
    // mlx90382_set_gpio_if(dev, 0);

	// mlx90382_register_read(dev, MLX90382_CRC_RESULT, buffer, 2);
	// LOG_INF("crc calculation done CRC = 0x%x\r\n", (buffer[0] << 8) + buffer[1]);

	// mlx90382_program_nvram(dev, 0x25A, 0x9a03);

	// mlx90382_read_nvram(dev, nvram_data, NVRAM_SIZE_WORDS);

	// mlx90382_frame_size(dev);

	// mlx90382_frame_read(dev, frame_buffer, ARRAY_SIZE(frame_buffer));
	// mlx90382_frame_read(dev, frame_buffer, ARRAY_SIZE(frame_buffer));
#endif

#if 0	
	mlx90382_set_faddr0(dev, MLX90382_LIN_PHASE);
	mlx90382_set_faddr1(dev, MLX90382_DRIFTC_PHASE);
	mlx90382_set_faddr2(dev, MLX90382_SC_PHASE);
	mlx90382_set_faddr3(dev, MLX90382_TEMP);
#else
	// mlx90382_set_faddr0(dev, 0);
	// mlx90382_set_faddr1(dev, 0);
	// mlx90382_set_faddr2(dev, 0);
	// mlx90382_set_faddr3(dev, 0);
#endif

	// frame_size = mlx90382_frame_size(dev);

	return 0;
}

static DEVICE_API(sensor, mlx90382_api) = {
	.attr_set     = mlx90382_attr_set,
	.attr_get     = mlx90382_attr_get,
	
	.sample_fetch = mlx90382_sample_fetch,
	.channel_get  = mlx90382_channel_get,
	
// #ifdef CONFIG_SENSOR_ASYNC_API
// 	.submit       = mlx90382_submit,
// 	.get_decoder  = mlx90382_get_decoder,
// #endif	
};

#define MLX90382_INIT(inst) \
	static struct mlx90382_data mlx90382_data_##inst; \
	static const struct mlx90382_config mlx90382_config_##inst = { \
		.spi = SPI_DT_SPEC_INST_GET(inst, \
				SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER), \
	}; \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, mlx90382_init, NULL, \
				 &mlx90382_data_##inst, &mlx90382_config_##inst, \
				 POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				 &mlx90382_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90382_INIT)
