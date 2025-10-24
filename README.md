# STM32C031C6 Drivers Driver Development

A bare-metal GPIO driver implementation for the STM32C031C6 microcontroller. Includes example applications demonstrating GPIO functionalities such as LED toggling, button handling, and interrupt management.
<br>
**Note:**
This project is under development not intended for production use.

## Overview

This project implements low-level GPIO drivers for the STM32C031C6 microcontroller and includes example applications demonstrating various GPIO functionalities.

## Hardware Requirements

- NUCLEO-C031C6 development board
- LED (for external LED examples)
- Push button (for external button examples)
- Jumper wires

## Project Structure
Workspace
(rerun without)
Collecting workspace information

I'll help you create a README.md file for your STM32C031C6 GPIO driver project. Here's a comprehensive README:

Project Structure

```.
Project Root
├── drivers
│   ├── Inc
│   │   ├── stm32c031xx.h           # MCU specific header
│   │   └── stm32c031xx_gpio_driver.h
│   └── Src
│       └── stm32c031xx_gpio_driver.c
├── Src
│   ├── 001LedToggle.c              # LED toggle example
│   ├── 002LedButton.c              # On-board button & LED example
│   ├── 003LedExternalBtn.c         # External button & LED example
│   ├── 004InterruptBtn.c           # GPIO interrupt 
└── Startup
    └── startup_stm32c031c6tx.s

```

## Example Applications

1. **LED Toggle (001LedToggle.c)**
   - Demonstrates basic GPIO output functionality
   - Toggles on-board LED (PA5) with a delay

2. **On-board Button & LED (002LedButton.c)**
   - Uses on-board user button (PC13) to control LED
   - Demonstrates input and output GPIO operations

3. **External Button & LED (003LedExternalBtn.c)**
   - Controls external LED (PB12) with external button (PA8)
   - Shows external GPIO device interfacing

4. **Interrupt Button (004InterruptBtn.c)**
   - Demonstrates GPIO interrupt functionality
   - Uses external interrupt (EXTI) for button press detection

## GPIO Features Implemented

- GPIO pin initialization and configuration
- Input/Output modes
- Pull-up/Pull-down configuration
- Output type selection (Push-pull/Open-drain)
- Interrupt configuration (Rising/Falling edge)
- Port read/write operations
- Atomic set/reset operations

## Building the Project

This project uses STM32CubeIDE for development. To build:

1. Import the project into STM32CubeIDE
2. Select the desired example file in the project explorer
3. Build the project using the hammer icon or Ctrl+B
4. Flash to the board using the Run button

## Pin Configurations

- On-board LED: PA5
- On-board User Button: PC13
- External LED: PB12
- External Button: PA8


## Contributing

Feel free to submit issues and pull requests to improve the drivers and examples.