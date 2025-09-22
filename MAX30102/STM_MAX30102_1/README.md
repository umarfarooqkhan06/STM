# STM32 MAX30102 Heart-Rate/SpO2 Application Skeleton

This folder mirrors the layout of a CubeIDE project so you can drop the
files directly into the project you already generated (PB8 = SCL, PB9 = SDA,
PA0 = INT).  Only the application-specific sources are included – keep the
HAL, CMSIS, startup, linker and `.ioc` files that CubeMX created for your
board.  **Do not replace your CubeMX-generated `Drivers` directory with the
placeholder contained here**; it exists only to preserve the folder structure
in git.

```
STM_MAX30102_1/
├── README.md
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── gpio.h
│   │   ├── i2c.h
│   │   ├── usart.h
│   │   ├── stm32f4xx_it.h
│   │   └── max30102_oximeter.h
│   └── Src/
│       ├── main.c
│       ├── max30102_oximeter.c
│       ├── stm32f4xx_hal_msp.c
│       ├── stm32f4xx_it.c
│       ├── gpio.c
│       ├── i2c.c
│       └── usart.c
├── Core/Startup/
│   └── startup_placeholder.txt
└── Drivers/
    └── README.md  ← reminder to keep the CubeMX HAL/CMSIS content
```

> ⚠️ Keep the HAL/CMSIS drivers, startup assembly, linker script and `.ioc`
> that CubeMX produced.  This repository only supplies the user code that you
> replace or add under the **Core** tree.  If those drivers disappear you'll
> see build errors such as `fatal error: stm32f4xx_hal_i2c.h: No such file or
> directory` – restore the CubeMX `Drivers` folder in that case.

## Integration steps

1. Generate your CubeMX project targeting the STM32 part/board you are using
   (ensure I2C1 SCL=PB8, SDA=PB9, and configure PA0 as an EXTI line on the
   falling edge).
2. Copy the files from `Core/Inc` and `Core/Src` into the matching folders in
   the CubeIDE project, replacing the generated `main.c`, `stm32f4xx_it.c`
   and peripheral init files (`gpio.c`, `i2c.c`, `usart.c`, `stm32f4xx_hal_msp.c`).
   Adjust `SystemClock_Config` if your board uses a different clock source.
3. Keep the rest of the Cube-generated artifacts (HAL drivers, startup, linker
   scripts, etc.).  The new `stm32f4xx_hal_conf.h` in this folder matches the
   modules used by the example, but you can retain your auto-generated copy if
   it already enables I2C/UART/GPIO.
4. Build and flash from CubeIDE.  The example streams formatted heart-rate,
   SpO₂ and die-temperature values over `USART2` every time a beat is detected.

The code configures the MAX30102 for multi-LED SpO₂ mode, consumes FIFO samples
as they arrive, performs DC removal, mean-difference and Butterworth filtering,
tracks pulses, and computes both BPM and SpO₂ using RMS ratios.  Die temperature
is queried on every poll so you can monitor sensor thermal behaviour.

