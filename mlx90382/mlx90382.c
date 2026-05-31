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

#define MLX90382_REG_ANGLE        0x3FFF
#define MLX90382_READ_BIT         BIT(14)
#define MLX90382_PARITY_BIT       BIT(15)
#define MLX90382_DATA_MASK        GENMASK(13, 0)
#define MLX90382_ERROR_BIT        BIT(14)
// #define MLX90382_MAX_STEPS        0x1000	//MLX90382AA/AB
#define MLX90382_MAX_STEPS        0x10000	//MLX90382BA
#define MLX90382_FULL_ANGLE_DEG   360
#define MLX90382_MICRO_DEGREE     1000000

#define MLX90382BA

#if defined MLX90382AA
#define MLX90382_DSP_BASE_ADDR              0x000
#define MLX90382_NVRAM_BASE_ADDR            0x100

#define MLX90382_CONFIG_REG                 (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define MLX90382_PHASE_OFS                  (MLX90382_NVRAM_BASE_ADDR + 0x20)   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)
#define MLX90382_PWM_PERIOD                 (MLX90382_NVRAM_BASE_ADDR + 0x2E)

#define MLX90382_SOFT_RESET                 0x004
#define MLX90382_TEMP                       0x03C
#define MLX90382_LIN_PHASE                  0x03E   //Angular value after linearization, resolution 360/216 deg; Writable only with LOCK_PHASE = 1; Before delay compensation
#define MLX90382_SPEED                      0x040
#define MLX90382_DRIFTC_PHASE               0x046   //Angular value after delay compensation and zero-point offset correction, resolution 360/0x10000 deg
#define MLX90382_SC_PHASE                   0x048   //Position value after signal conditioning
#define MLX90382_GC_I                       0x04A   //Gain compensated I component
#define MLX90382_GC_Q                       0x04E   //Gain compensated Q component

#define MLX90382_ANA_VERSION                0x0EE
#define MLX90382_DIG_VERSION                0x0F0
#elif defined MLX90382AB
#define MLX90382_DSP_BASE_ADDR              0x000
#define MLX90382_NVRAM_BASE_ADDR            0x100

#define NVRAM_NVOP_KEY_ADDRESS              ((uint16_t)0x10)
#define NVRAM_STORE_REQ_ADDRESS             ((uint16_t)0x13)
#define NVRAM_STORE_REQ_INDEX_W             ((uint16_t)0x8)

#define NVRAM_ADDR_BEGIN                    (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define NVRAM_ADDR_END                      (MLX90382_NVRAM_BASE_ADDR + 0x60)

/** \brief Size of the NVRAM in amount of words.*/
#define NVRAM_SIZE_WORDS                         ((NVRAM_ADDR_END - NVRAM_ADDR_BEGIN) / 2)

#define MLX90382_CONFIG_REG                 (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define MLX90382_PHASE_OFS                  (MLX90382_NVRAM_BASE_ADDR + 0x20)   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)
#define MLX90382_PWM_PERIOD                 (MLX90382_NVRAM_BASE_ADDR + 0x2E)
#define MLX90382_CUS_CRC_ADDR               (MLX90382_NVRAM_BASE_ADDR + 0x5E)

#define MLX90382_SOFT_RESET                 0x004
#define MLX90382_APPLICATION_MODE           0x024
#define MLX90382_CRC_RESULT           		0x026
#define MLX90382_CRC_CALC           		0x028
#define MLX90382_TEMP                       0x03C
#define MLX90382_LIN_PHASE                  0x03E   //Angular value after linearization, resolution 360/216 deg; Writable only with LOCK_PHASE = 1; Before delay compensation
#define MLX90382_SPEED                      0x040
#define MLX90382_DRIFTC_PHASE               0x046   //Angular value after delay compensation and zero-point offset correction, resolution 360/0x10000 deg
#define MLX90382_SC_PHASE                   0x048   //Position value after signal conditioning
#define MLX90382_GC_I                       0x04A   //Gain compensated I component
#define MLX90382_GC_Q                       0x04E   //Gain compensated Q component

#define MLX90382_ANA_VERSION                0x0EE
#define MLX90382_DIG_VERSION                0x0F0

#define MLX90382_DEV_INFO                	0x10E

#elif defined MLX90382BA
#define MLX90382_DSP_BASE_ADDR              0x000
#define MLX90382_NVRAM_BASE_ADDR            0x200

#define NVRAM_NVOP_KEY_ADDRESS              ((uint16_t)0x10)
#define NVRAM_STORE_REQ_ADDRESS             ((uint16_t)0x13)
#define NVRAM_STORE_REQ_INDEX_W             ((uint16_t)0x8)

#define NVRAM_ADDR_BEGIN                    (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define NVRAM_ADDR_END                      (MLX90382_NVRAM_BASE_ADDR + 0x60)

/** \brief Size of the NVRAM in amount of words.*/
#define NVRAM_SIZE_WORDS                         ((NVRAM_ADDR_END - NVRAM_ADDR_BEGIN) / 2)

#define MLX90382_CONFIG_REG                 (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define MLX90382_PHASE_OFS                  (MLX90382_NVRAM_BASE_ADDR + 0x20)   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)
#define MLX90382_PWM_PERIOD                 (MLX90382_NVRAM_BASE_ADDR + 0x2E)
#define MLX90382_CUS_CRC_ADDR               (MLX90382_NVRAM_BASE_ADDR + 0x5E)

#define MLX90382_SOFT_RESET                 0x004
#define MLX90382_APPLICATION_MODE           0x024
#define MLX90382_CRC_RESULT           		0x026
#define MLX90382_CRC_CALC           		0x028
#define MLX90382_TEMP                       0x038
#define MLX90382_LIN_PHASE                  0x03A   //Angular value after linearization, resolution 360/216 deg; Writable only with LOCK_PHASE = 1; Before delay compensation
#define MLX90382_SPEED                      0x03C
#define MLX90382_DRIFTC_PHASE               0x042   //Angular value after delay compensation and zero-point offset correction, resolution 360/0x10000 deg
#define MLX90382_SC_PHASE                   0x044   //Position value after signal conditioning
#define MLX90382_GC_I                       0x048   //Gain compensated I component
#define MLX90382_GC_Q                       0x04E   //Gain compensated Q component

#define MLX90382_ANA_VERSION                0x0EE
#define MLX90382_DIG_VERSION                0x0F0

#define MLX90382_DEV_INFO                	0x10E
#endif

enum MLX90382_CMD
{
    CMD_RR  = 0xCC,
    CMD_RW  = 0x78,
    CMD_FR  = 0x34,
    CMD_SFR = 0xD0, 
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

union mlx90382_0x024_reg
{
    uint16_t word_val;

    struct
    {
        uint8_t application_mode     	: 2;    //BIT0-BIT2
        uint8_t disable_crc_calc   		: 1;
        uint16_t not_used           	: 13;
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

static int mlx90382_register_read(const struct device *dev, uint16_t reg_addr, uint8_t *buffer, size_t reg_size)
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

	buffer_tx[0] = (uint8_t)CMD_RR | (reg_addr>>9);
	buffer_tx[1] = (reg_addr>>1) & 0xFF;
	buffer_tx[2] = 0x00;
	buffer_tx[3] = 0x00;

	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

	if (ret != 0) {
		LOG_ERR("%s: spi_transceive failed with error %i", dev->name, ret);
		return ret;
	}

	for (uint8_t i = 0; i < reg_size; i++) {
		buffer[i] = buffer_rx[3 + i];
	}

	// LOG_INF("spi: 0x%x config: 0x%x 0x%x 0x%x 0x%x 0x%x", cfg->spi, cfg->spi.config.frequency, cfg->spi.config.operation, cfg->spi.config.slave, cfg->spi.config.cs.gpio, cfg->spi.config.cs);
	LOG_INF("tx: 0x%x 0x%x 0x%x 0x%x", buffer_tx[0], buffer_tx[1], buffer_tx[2], buffer_tx[3]);
	LOG_INF("rx: 0x%x 0x%x 0x%x 0x%x 0x%x", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3], buffer_rx[4]);

	return ret;
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

	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

	if (ret != 0) {
		LOG_ERR("%s: spi_transceive failed with error %i", dev->name, ret);
		return ret;
	}

	printk("mlx90382_register_write: reg_addr=0x%04x, reg_val=0x%04x\r\n", reg_addr, reg_val);
	printk("tx: 0x%x 0x%x 0x%x 0x%x\r\n", buffer_tx[0], buffer_tx[1], buffer_tx[2], buffer_tx[3]);
	printk("rx: 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", buffer_rx[0], buffer_rx[1], buffer_rx[2], buffer_rx[3], buffer_rx[4]);

	return ret;
}

int mlx90382_get_crc(const struct device *dev, uint16_t *crc)
{
	int rec = 0;
	union mlx90382_crc_calc_reg crc_calc_reg;

	// crc_calc_reg.word_val = 0x0000;
	// crc_calc_reg.crc_calc_strt = 0x0001;
	mlx90382_register_write(dev, MLX90382_CRC_CALC, 0x0000);
	mlx90382_register_write(dev, MLX90382_CRC_CALC, 0x0001);

	k_msleep(1000);

	do 
	{
		mlx90382_register_read(dev, MLX90382_CRC_CALC, (uint8_t *)&crc_calc_reg, 2);

		printf("crc_calc_reg = 0x%x\r\n", crc_calc_reg.word_val);
	}
	while (crc_calc_reg.word_val == 0x0000);

	// while (mlx90382_register_read(dev, MLX90382_CRC_CALC, (uint8_t *)&crc_calc_reg, 2) == 0)
	// {
	// 	// if (crc_calc_reg.crc_calc_done == 0x0001)
	// 	if (crc_calc_reg.word_val != 0x000)
	// 	{
	// 		printf("crc_calc_reg = 0x%x\r\n", crc_calc_reg.word_val);

	// 		break;
	// 	}

	// 	printf("claculating crc... crc_cal_reg = 0x%x\r\n", crc_calc_reg.word_val);
	// }

	mlx90382_register_read(dev, MLX90382_CRC_RESULT, (uint8_t *)crc, 2);
	printf("crc_calc_reg = 0x%x\r\n", crc_calc_reg.word_val);

	return rec;
}

int mlx90382_set_crc(const struct device *dev, uint16_t crc)
{
	int rec = 0;

	printf("set crc in nvram[0x%x]: 0x%x\r\n", MLX90382_CUS_CRC_ADDR, crc);
	rec = mlx90382_register_write(dev, MLX90382_CUS_CRC_ADDR, crc);

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

	printf("crc_ccitt_update: crc = 0x%x\r\n", crc);
    
	return crc & 0xffff;
}

int mlx90382_read_nvram(const struct device *dev, uint16_t *data_rx, uint16_t n)
{
	int res = 0;

	uint8_t buf[2];

	mlx90382_set_application_mode(dev, 2);
	
	for (uint16_t i = 0; i < n; i++)
	{
		res = mlx90382_register_read(dev, MLX90382_NVRAM_BASE_ADDR + i*2, buf, 2);
		
		data_rx[i] = (buf[0]<<8) + buf[1];
		// data_rx[i+1] = buf[1];

		printf("NVRAM[0x%04x] = 0x%04x\r\n", MLX90382_NVRAM_BASE_ADDR + i*2, data_rx[i]);
	}

	mlx90382_soft_reset(dev);

    if (crc_ccitt_update(0xffff, data_rx, NVRAM_SIZE_WORDS) != 0) 
	{
		printf("CRC check failed for NVRAM data\r\n");
		return -1;
    } 
	else 
	{
		printf("CRC check passed for NVRAM data\r\n");
    }	
}

int mlx90382_program_nvram(const struct device *dev, uint16_t addr, uint16_t val)
{
	int res = 0;
    uint16_t crc;

	res = mlx90382_register_write(dev, 0x0BE, 3<<1);
	res = mlx90382_register_write(dev, 0x024, 6);

	// mlx90382_set_application_mode(dev, 2);

	res = mlx90382_register_read(dev, MLX90382_CUS_CRC_ADDR, (uint8_t *)crc, 2);
	printf("current crc in nvram: 0x%x\r\n", crc);

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
    int res=0;
    uint8_t buf[2];

    union mlx90382_config_reg reg;

    mlx90382_register_read(dev, MLX90382_CONFIG_REG, buf, 2);

    reg.word_val = (buf[0]<<8) + buf[1];

    reg.sensing_mode = mode;
    reg.gpio_if = 0x2;

	reg.word_val = 0xe728;

    // res = mlx90382_register_write(dev, MLX90382_CONFIG_REG, reg.word_val);
	res = mlx90382_program_nvram(dev, MLX90382_CONFIG_REG, reg.word_val);

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

    res = mlx90382_register_read(dev, MLX90382_CONFIG_REG, buf, 2);
    if (res != 0)
    {
		LOG_ERR("mlx90382_get_config is failed\r\n");

        return res;
    }

    config->word_val = ((buf[0] << 8) + buf[1]);
	// printf("mlx90382_get_config: 0x%04x\r\n", config->word_val);
	
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
    // printf("mlx90382_get_analog_version: 0x%04x\r\n", *version);

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
	// printf("mlx90382_get_digital_version: 0x%04x\r\n", *version);

    return 0;
}

int mlx90382_get_zero_position(const struct device *dev, uint16_t *position)
{
    int res = 0;
    uint8_t buf[2];

    res = mlx90382_register_read(dev, MLX90382_PHASE_OFS, buf, 2);
    if (res != 0)
    {
        printf("mlx90382_get_zero_position is failed\r\n");
        return res;
    }

    *position = (buf[0]<<8) + buf[1];
    // printf("mlx90382_get_zero_position: 0x%04x\r\n", *position);

    return 0;
}

int mlx90382_get_info(const struct device *dev)
{
    int result = 0;

    uint16_t a_ver;
    uint16_t d_ver_h;
    uint32_t d_ver;
    uint16_t zero_position;
    union mlx90382_config_reg config;

    mlx90382_get_config(dev, &config);
    mlx90382_get_zero_position(dev, &zero_position);
    mlx90382_get_digital_version(dev, &d_ver_h);
    mlx90382_get_analog_version(dev, &a_ver);
    d_ver = (a_ver>>8) + (d_ver_h<<16);

    printf("aversion:%x\n", a_ver&0xFF);
    printf("dversion:%x\n", d_ver);
    printf("zeroposition:%x\n", zero_position);
    printf("config:%x\n", config.word_val);

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

	res = mlx90382_program_nvram(dev, MLX90382_PHASE_OFS, position);

    if (res != 0)
    {
        printf("mlx90382_set_zero_position is failed\r\n");
    }

    return res;
}

int mlx90382_set_gpio_if(const struct device *dev, uint8_t val)
{
    int ret = 0;
    uint8_t buf[4];

    union mlx90382_config_reg reg;

    mlx90382_register_read(dev, MLX90382_CONFIG_REG, buf, 2);

    reg.word_val = (buf[0]<<8) + buf[1];

    reg.gpio_if = val;

    ret = mlx90382_register_write(dev, MLX90382_CONFIG_REG, reg.word_val);

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

	uint8_t buffer[2];

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
			angle_deg = (angle_deg / MLX90382_MAX_STEPS) * MLX90382_FULL_ANGLE_DEG;
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

static int mlx90382_init(const struct device *dev)
{
	uint8_t buffer[4];

    uint16_t nvram_data[NVRAM_SIZE_WORDS];

	const struct mlx90382_config *cfg = dev->config;

	LOG_INF("mlx90382_init");

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	mlx90382_register_read(dev, MLX90382_CONFIG_REG, buffer, 2);
	mlx90382_register_read(dev, MLX90382_ANA_VERSION, buffer, 2);
	mlx90382_register_read(dev, MLX90382_DIG_VERSION, buffer, 2);

	mlx90382_register_read(dev, MLX90382_DEV_INFO, buffer, 2);
	printf("device info: 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);

	mlx90382_register_write(dev, MLX90382_APPLICATION_MODE, 0);

	mlx90382_register_read(dev, MLX90382_APPLICATION_MODE, buffer, 2);
	printf("application mode: 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);

	mlx90382_register_read(dev, 0xBE, buffer, 2);
	printf("[0xBE] = 0x%04x\r\n", (buffer[0]<<8) + buffer[1]);	

	// mlx90382_register_read(dev, MLX90382_TEMP, buffer, 2);
	// mlx90382_register_read(dev, MLX90382_SC_PHASE, buffer, 2);

    // set GPIO_IF as SPI bus mode
    // mlx90382_set_gpio_if(dev, 2);

	// mlx90382_register_read(dev, MLX90382_CRC_RESULT, buffer, 2);
	// printf("crc calculation done CRC = 0x%x\r\n", (buffer[0] << 8) + buffer[1]);

	mlx90382_read_nvram(dev, nvram_data, NVRAM_SIZE_WORDS);

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
