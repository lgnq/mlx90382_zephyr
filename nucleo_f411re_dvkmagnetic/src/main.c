/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>

#include <zephyr/drivers/sensor/mlx90382.h>

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct device *const encoder = DEVICE_DT_GET(DT_ALIAS(encoder0));

enum CMD
{
    RT_SENSOR_CTRL_USER_CMD_SOFT_RESET = 0x101,
    RT_SENSOR_CTRL_USER_CMD_INFO,
    RT_SENSOR_CTRL_USER_CMD_LIN_PHASE,
    RT_SENSOR_CTRL_USER_CMD_DRIFTC_PHASE,
    RT_SENSOR_CTRL_USER_CMD_SC_PHASE,
    RT_SENSOR_CTRL_USER_CMD_SPEED,
    RT_SENSOR_CTRL_USER_CMD_TEMP,
    RT_SENSOR_CTRL_USER_CMD_GET_ZEROPOSITION,
    RT_SENSOR_CTRL_USER_CMD_SET_ZEROPOSITION,
    RT_SENSOR_CTRL_USER_CMD_SET_SENSING_MODE,
};

uint16_t sample_freq = 100;
static bool sample_on = false;

static int measurement_on(const struct shell *shell, size_t argc, char *argv[])
{
	printk("measurement on\n");
	sample_on = true;
 
    return 0;
}

static int measurement_off(const struct shell *shell, size_t argc, char *argv[])
{
	printk("measurement off\n");
	sample_on = false;

    return 0;
}

int8_t mlx90382_ops_ctrl_fn(const struct shell *shell, size_t argc, char *argv[])
{
    int8_t result = 0;

	int32_t cmd = -1;

	cmd = strtol(argv[1], NULL, 10);
    uint16_t args = atoi(argv[2]);

	printk("mlx90382 ops ctrl is %d args = %d\r\n", cmd, args);
	
	switch (cmd) 
    {
    case RT_SENSOR_CTRL_USER_CMD_INFO:
        result = mlx90382_get_info(encoder);
        break;
    case RT_SENSOR_CTRL_USER_CMD_LIN_PHASE:
        // result = mlx90382_get_lin_phase(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_DRIFTC_PHASE:
        // result = mlx90382_get_driftc_phase(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SC_PHASE:
        // result = mlx90382_get_sc_phase(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SPEED:
        // result = mlx90382_get_speed(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_TEMP:
        // result = mlx90382_get_temp(mlx_dev, (float *)args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SOFT_RESET:
        result = mlx90382_soft_reset(encoder);
        break;
    case RT_SENSOR_CTRL_USER_CMD_GET_ZEROPOSITION:
        result = mlx90382_get_zero_position(encoder, &args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_ZEROPOSITION:
        result = mlx90382_set_zero_position(encoder, args);
        break;
    case RT_SENSOR_CTRL_USER_CMD_SET_SENSING_MODE:
        result = mlx90382_set_sensing_mode(encoder, args);
        break;
    default:
        printf("unknown MLX90382 CTRL CMD\r\n");
        return -1;
    }

    return result;
}

int mlx90382_set_sample_freq_fn(const struct shell *shell, size_t argc, char *argv[])
{
    int res = 0;

    sample_freq = atoi(argv[1]);
    printf("sample freq = %d\r\n", sample_freq);

    return res;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    mlx90382_measurement_onoff_cmds,
    SHELL_CMD_ARG(on, NULL, "start measurement\n", measurement_on, 1, 0),
    SHELL_CMD_ARG(off, NULL, "stop measurement\n", measurement_off, 1, 0),
    SHELL_SUBCMD_SET_END
    );

SHELL_CMD_REGISTER(mlx90382_measurement_onoff, &mlx90382_measurement_onoff_cmds, "Set mlx90382 measurement on/off", NULL);

SHELL_CMD_REGISTER(mlx90382_ops_ctrl, NULL, "Set mlx90382 operations control", mlx90382_ops_ctrl_fn);	
SHELL_CMD_REGISTER(mlx90382_set_sample_freq, NULL, "Set mlx90382 sample frequency", mlx90382_set_sample_freq_fn);	

int main(void)
{
	int ret;
	struct sensor_value value;

	float lin_phase = 0;
    float driftc_phase = 0;
    float sc_phase = 0;
    float speed = 0;
    float temp = 0;

	if (!device_is_ready(encoder)) 
	{
		printk("sensor: device not ready.\n");
		return 0;
	}

	while (1) 
	{		
		if (sample_on) 
		{
			ret = sensor_sample_fetch(encoder);
			if (ret) 
			{
				printk("sensor_sample_fetch failed ret %d\n", ret);
				return 0;
			}

            ret = sensor_channel_get(encoder, SENSOR_CHAN_LIN_PHASE, &value);
			lin_phase = sensor_value_to_float(&value);

			ret = sensor_channel_get(encoder, SENSOR_CHAN_DRIFTC_PHASE, &value);
			driftc_phase = sensor_value_to_float(&value);

			ret = sensor_channel_get(encoder, SENSOR_CHAN_SC_PHASE, &value);
			sc_phase = sensor_value_to_float(&value);

			ret = sensor_channel_get(encoder, SENSOR_CHAN_DIE_TEMP, &value);
			temp = sensor_value_to_float(&value);
			// printk("die temperature = ( %f )\n", sensor_value_to_double(&value));

            printf("data:%.3f,%.3f,%.3f,%.3f,%.3f\n", lin_phase, driftc_phase, sc_phase, speed, temp);
		}

		k_msleep(sample_freq);
	}

	return 0;
}
