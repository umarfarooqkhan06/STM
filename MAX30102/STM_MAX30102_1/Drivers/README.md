# CubeMX drivers placeholder

This repository does **not** ship the STM32 HAL/CMSIS driver sources. When you
copy the `Core` directory into your own CubeIDE project, keep the `Drivers`
folder that CubeMX generated (it contains `STM32F4xx_HAL_Driver`, `CMSIS`, the
startup assembly file, linker scripts, etc.).

If you accidentally replaced your project `Drivers` directory with this one,
regenerate the CubeMX project or copy the folder back from a fresh CubeIDE
project so that files such as `stm32f4xx_hal_i2c.h` are available to the
compiler.
