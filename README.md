# UART-DRIVER (Bare-Metal STM32)

A minimal bare-metal UART driver for STM32 enabling MCU↔PC communication via direct register bit manipulation (no HAL). Includes clock/baud init, TX/RX (polling), and simple echo demo.

## Hardware
- Board: NUCLEO-F767ZI
- UART: USART3 (PD8=TX, PD9=RX) → ST-LINK VCP @ 115200 8-N-1

## Build & Flash
```bash
make
make flash
