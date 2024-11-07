# 2-DOF-Parallel-Robotic-Arm

This project implements an 4 layer PCB with built in Dual degree of freedom parallel robotic arm motor system with STM32G030F6P6.

## Project Overview

This project utilizes an STM32G030F6P6 microcontroller to control a 2-DOF motor system. The system is designed for efficient, real-time kinematic control with the following key features:

- **EXTI Boundary Detection**: Uses Hall sensors with EXTI to detect physical boundaries and reset motor positions.
- **Precomputed Lookup Tables**: Stores angle-to-step mappings in Flash to enable quick angle retrieval and minimize in-operation computations.
- **Real-time UART Control**: Allows external updates to motor positions using UART for adaptable control.
- **Bitwise Manipulation for Motor Control**: Manages motor rotation using bitwise shifts and masks to streamline resource use.

## PCB Design Overview

- **MCU**: The board is powered by the STM32G030F6P6 microcontroller, which provides EXTI (External Interrupt) and DMA (Direct Memory Access) functionalities critical for handling sensor inputs and efficient motor control.
- **Motor Driver and Stepper Motors**: The system drives two 28BYJ-48 unipolar stepper motors, each controlled via the ULN2003AN motor driver. 
  - The ULN2003AN drivers are managed by a 74HC595D shift register, which receives serial input from the MCU and outputs parallel signals to the motor driver, allowing synchronous step control.
- **Power Management**: A robust power management section supplies stable voltages to each part of the system:
  - A buck converter using AOZ1280CI steps down 12V to 3.3V, supplying the MCU and other logic-level components.
  - Two debug LEDs indicate active 3.3V and 12V power supplies, providing visual status indicators for quick troubleshooting.
  - A jumper provides an option to individually supply 3.3V to the logic section, allowing for isolated logic debugging as needed.
- **User Interface and Debugging**: 
  - Four push buttons are incorporated, allowing manual control for adjusting motor axis positions directly.
  - Two 3-pin male connectors are available for Hall sensor inputs, facilitating feedback-based boundary detection and control.
  - SWD (Serial Wire Debug) pins are included for in-circuit debugging, along with UART TX and RX pins for data communication and logging.
- **Mechanical Integration**: 
  - The board measures 43mm by 58mm and includes four M3 mounting holes, allowing secure attachment to a dedicated surface for stability during motor operations.

## Core Embedded System Components and Functionality

1. **Motor Initialization and Setup**  
   Initializes two stepper motors (`motor1` and `motor2`) and an arm struct, configuring each with defined starting positions and kinematic parameters.

2. **Motor Control and Step Calculation**
   - `rotate()`: Adjusts motor positions by incrementing or decrementing position values, with bitwise shifts controlling left and right motor rotation.
   - `get_step()`: Computes required steps for (x, y) coordinates using the lookup table, enabling fast conversions from Cartesian coordinates to motor steps.

3. **Boundary Detection and Zeroing**
   - Uses EXTI for detecting motor boundary limits via Hall sensors, resetting motors to zero positions as needed by iterating incremental steps.

4. **UART Communication for Real-time Updates**
   - `uart_to_motor()`: Receives (x, y) coordinates over UART and translates these into motor steps using `get_step()` and `rotate()`, facilitating dynamic, real-time external control.

5. **Shift Register and GPIO Control**
   - `shiftIn()`: Manages GPIO pins to interface with a shift register, controlling motor outputs based on serialized input values.


