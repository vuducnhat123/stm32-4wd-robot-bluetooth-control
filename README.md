# stm32-4wd-robot-bluetooth-control
# STM32 4WD Robot Control via Bluetooth

## Description
This project implements a 4-wheel robot control system using STM32 and Bluetooth communication.

## Features
- Control 4 DC motors using PWM
- Receive commands via Bluetooth (HC-05)
- UART communication between STM32 and Bluetooth module
- Encoder feedback for speed measurement
- PID controller for stable motor speed

## Hardware
- STM32F405
- HC-05 Bluetooth module
- DC motors (4WD)
- Motor driver (L298N or BTS7960)
- Encoder

## Control Flow
Controller → Bluetooth → STM32 → Motor Driver → Motor

## Result
- Real-time control
- Stable speed
- Smooth movement
