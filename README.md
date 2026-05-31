# MLX90382 sensor package for Zephyr

## Overview

**mlx90382_zephyr** is Melexis MLX90382 sensor package for Zephyr, based on v4.4.0.

## Description

This **cmsis_device_l4** MCU component repo is one element of the STM32CubeL4 MCU embedded software package, providing the **cmsis device** part.

## Release note

Details about the content of this release are available in the release note [here](https://htmlpreview.github.io/?https://github.com/STMicroelectronics/cmsis_device_l4/blob/master/Release_Notes.html).

## Compatibility information

MLX90384AA, MLX90384AB, MLX90384BA 

## Troubleshooting
If you have any issue with the **Software content** of this repo, you can [file an issue on Github](https://github.com/STMicroelectronics/cmsis_device_l4/issues/new).

For any other question related to the product, the tools, the environment, you can submit a topic on the [ST Community/STM32 MCUs forum](https://community.st.com/s/group/0F90X000000AXsASAW/stm32-mcus).

## Howto add mlx90382 sensor package into Zephyr
1, copy mlx90382.h to "zephyr\include\zephyr\drivers\sensor"

2, copy melexis,mlx90382.yaml to "zephyr\dts\bindings\sensor"

3, copy mlx90382 to "zephyr\drivers\sensor\melexis\"

4, update zephyr\drivers\sensor\melexis\CMakeLists.txt
# Copyright (c) 2024 Florian Weber <Florian.Weber@live.de>
# SPDX-License-Identifier: Apache-2.0

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_MLX90382 mlx90382)
add_subdirectory_ifdef(CONFIG_MLX90384 mlx90384)
add_subdirectory_ifdef(CONFIG_MLX90394 mlx90394)
add_subdirectory_ifdef(CONFIG_MLX90396 mlx90396)
# zephyr-keep-sorted-stop

5, update zephyr\drivers\sensor\melexis\Kconfig
# Copyright (c) 2024 Florian Weber <Florian.Weber@live.de>
# SPDX-License-Identifier: Apache-2.0

# zephyr-keep-sorted-start
source "drivers/sensor/melexis/mlx90382/Kconfig"
source "drivers/sensor/melexis/mlx90384/Kconfig"
source "drivers/sensor/melexis/mlx90394/Kconfig"
source "drivers/sensor/melexis/mlx90396/Kconfig"
# zephyr-keep-sorted-stop

## Howto compile mlx90382 application
west build -p always -b nucleo_f411re nucleo_f411re_dvkmagnetic -DDTC_OVERLAY_FILE="MLX90382BA.overlay"

## Howto upgrade the flash
west flash

