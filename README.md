# MotorController Library for ESP32 Arduino

A lightweight motor controller library for the TB6612FNG dual DC motor driver using ESP32's LEDC peripheral for PWM generation.

## Features

- **Efficient PWM Control**: Uses ESP32's LEDC peripheral for precise speed control
- **Bidirectional Operation**: Simple control of motor direction and speed
- **Multiple Stop Modes**: Support for both soft stop (coasting) and hard stop (braking)
- **Fault Detection**: Optional fault input monitoring for motor protection
- **Configurable Resolution**: Adjustable PWM resolution from 8 to 14 bits
- **Simple Arduino API**: Easy-to-use interface with familiar Arduino functions

## Hardware Requirements

- ESP32 microcontroller
- TB6612FNG motor driver
- DC motor(s)

## Installation

1. Download the library as a ZIP file
2. In the Arduino IDE, go to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file
4. The library will be installed and available in the Arduino IDE

## Simple Usage Example

```cpp
#include <MotorController.h>

using namespace MotorControl;

// Define GPIO pins
const int MOTOR_IN1 = 5;
const int MOTOR_IN2 = 6;
const int MOTOR_PWM = 7;
const int CHANNEL   = 0;

// Create motor controller instance
MotorController motor(MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, CHANNEL);

void setup() {
  // Initialize with 10kHz PWM frequency and 10-bit resolution
  motor.begin(10000, 10);
}

void loop() {
  // Forward at 50% speed
  motor.motorGo(512);
  delay(2000);

  // Stop
  motor.softStop();
  delay(1000);

  // Backward at 50% speed
  motor.motorGo(-512);
  delay(2000);

  // Brake
  motor.hardStop();
  delay(1000);
}
```
