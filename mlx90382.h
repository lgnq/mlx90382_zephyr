#ifndef __MLX90382_H__
#define __MLX90382_H__

#include <zephyr/device.h>

#define MLX90382BA

#if defined MLX90382AA
#define MLX90382_DSP_BASE_ADDR              0x000
#define MLX90382_NVRAM_BASE_ADDR            0x100

#define MLX90382_NV_CONFIG_REG                 (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define MLX90382_NV_PHASE_OFS                  (MLX90382_NVRAM_BASE_ADDR + 0x20)   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)
#define MLX90382_NV_PWM_PERIOD                 (MLX90382_NVRAM_BASE_ADDR + 0x2E)

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

#define MLX90382_NV_CONFIG_REG                 (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define MLX90382_NV_PHASE_OFS                  (MLX90382_NVRAM_BASE_ADDR + 0x20)   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)
#define MLX90382_NV_PWM_PERIOD                 (MLX90382_NVRAM_BASE_ADDR + 0x2E)
#define MLX90382_NV_CUS_CRC_ADDR               (MLX90382_NVRAM_BASE_ADDR + 0x5E)

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

#define NVRAM_SIZE_WORDS                    ((NVRAM_ADDR_END - NVRAM_ADDR_BEGIN) / 2)

#define MLX90382_NV_CONFIG_REG              (MLX90382_NVRAM_BASE_ADDR + 0x00)
#define MLX90382_NV_PHASE_OFS               (MLX90382_NVRAM_BASE_ADDR + 0x20)   //Phase/Angle offset before signal conditioning, resolution 360/216 deg (signed 2th-complement)
#define MLX90382_NV_SC_X1             		(MLX90382_NVRAM_BASE_ADDR + 0x22)
#define MLX90382_NV_SC_X2             		(MLX90382_NVRAM_BASE_ADDR + 0x24)
#define MLX90382_NV_SC_Y1             		(MLX90382_NVRAM_BASE_ADDR + 0x26)
#define MLX90382_NV_SC_Y2             		(MLX90382_NVRAM_BASE_ADDR + 0x28)
#define MLX90382_NV_SC_YE             		(MLX90382_NVRAM_BASE_ADDR + 0x2A)
#define MLX90382_NV_SC_HL             		(MLX90382_NVRAM_BASE_ADDR + 0x2C)
#define MLX90382_NV_PWM_PERIOD              (MLX90382_NVRAM_BASE_ADDR + 0x2E)
#define MLX90382_NV_SPI_FADDR01             (MLX90382_NVRAM_BASE_ADDR + 0x30)
#define MLX90382_NV_SPI_FADDR23             (MLX90382_NVRAM_BASE_ADDR + 0x32)
#define MLX90382_NV_SPI_CONFIG              (MLX90382_NVRAM_BASE_ADDR + 0x34)
#define MLX90382_NV_CUS_CRC_ADDR            (MLX90382_NVRAM_BASE_ADDR + 0x5E)

#define MLX90382_SOFT_RESET                 0x004
#define MLX90382_APPLICATION_MODE           0x024
#define MLX90382_CRC_RESULT           		0x026
#define MLX90382_CRC_CALC           		0x028
#define MLX90382_STATE           			0x02C
#define MLX90382_DIAG1           		    0x02E
#define MLX90382_DIAG2           		    0x030
#define MLX90382_DIAG3           		    0x032
#define MLX90382_AGC_GAIN                   0x034
#define MLX90382_AGC_AMP                    0x036
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

/** @brief Sensor specific channels of mlx90382. */
enum mlx90382_channel{
    /** Linear phase */
    SENSOR_CHAN_LIN_PHASE = SENSOR_CHAN_PRIV_START,
    SENSOR_CHAN_DRIFTC_PHASE,
    SENSOR_CHAN_SC_PHASE,
};

int mlx90382_soft_reset(const struct device *dev);
int mlx90382_get_info(const struct device *dev);
int mlx90382_set_zero_position(const struct device *dev, uint16_t position);
int mlx90382_get_zero_position(const struct device *dev, uint16_t *position);
int mlx90382_set_sensing_mode(const struct device *dev, uint16_t mode);
int mlx90382_set_application_mode(const struct device *dev, uint16_t onoff);
int mlx90382_set_gpio_if(const struct device *dev, uint8_t val);
int mlx90382_shadow_register_monitor_onoff(const struct device *dev, uint8_t onoff);
int mlx90382_set_faddr0(const struct device *dev, uint8_t addr);
int mlx90382_set_faddr1(const struct device *dev, uint8_t addr);
int mlx90382_set_faddr2(const struct device *dev, uint8_t addr);
int mlx90382_set_faddr3(const struct device *dev, uint8_t addr);
int mlx90382_set_frfs(const struct device *dev, uint8_t frfs);
int mlx90382_set_frfsen(const struct device *dev, uint8_t onoff);
int mlx90382_set_frcrcen(const struct device *dev, uint8_t onoff);
int mlx90382_set_frinv(const struct device *dev, uint8_t inv);
int mlx90382_read_nvram(const struct device *dev, uint16_t *data_rx, uint16_t n);
int mlx90382_nvram_crc_update(const struct device *dev);
int mlx90382_set_sc_x1(const struct device *dev, uint16_t val);
int mlx90382_set_sc_x2(const struct device *dev, uint16_t val);
int mlx90382_set_sc_y1(const struct device *dev, uint16_t val);
int mlx90382_set_sc_y2(const struct device *dev, uint16_t val);
int mlx90382_set_sc_ye(const struct device *dev, uint16_t val);
int mlx90382_set_sc_hl(const struct device *dev, uint16_t val);

#endif /* __MLX90382_H__ */