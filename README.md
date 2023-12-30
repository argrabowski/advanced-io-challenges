# Real-Time Embedded Systems with Advanced I/O

## Overview

This lab explores advanced I/O concepts, including the use of PWM signals for audio playback and optimizing ADC functionality with DMA.

## Lab Components

### Challenge #1: Optimizing ADC Functionality with DMA

- Replaced ISR functionality with DMA to improve efficiency.
- Modified ADC initialization code to use two halves of the ADC buffer with alternating DMA channels.
- Demonstrated significant reduction in CPU load, allowing for more efficient system operation.

### Challenge #2: Capture Mode Timer for Frequency Measurement

- Implemented a capture mode timer to measure the period of an input square wave.
- Converted period measurements to frequencies and displayed them on the board.

### Challenge #3: Audio Playback using PWM

- Utilized PWM signals as digital-to-analog converters for audio playback.
- Adjusted PWM initialization code to create a PWM signal for this purpose.
- Implemented a PWM ISR to update the frequency of the signal for sound control.

## Code Structure

- `main.c`: Main source code file containing the initialization and setup of the lab components.
- `peripherals.h`: Header file containing function prototypes and necessary includes for peripherals.
- Other necessary header files and libraries.

## Setup and Usage

1. Clone the repository: `git clone https://github.com/argrabowski/advanced-io-challenges`
2. Open the project in your preferred development environment (e.g., Code Composer Studio, Keil, etc.).
3. Build and flash the code onto the EK-TM4C1294XL LaunchPad.

## Dependencies

- TI-RTOS
- TivaWare Peripheral Driver Library
