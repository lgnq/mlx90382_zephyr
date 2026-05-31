#ifndef __MLX90382_H__
#define __MLX90382_H__

#include <zephyr/device.h>

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

#endif /* __MLX90382_H__ */