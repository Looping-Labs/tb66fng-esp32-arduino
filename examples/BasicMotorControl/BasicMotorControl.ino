/**
 * @file BasicMotorControl.ino
 * @brief Basic example of motor control using the MotorController library
 *
 * This example demonstrates basic motor control functions including:
 * - Forward/backward rotation
 * - Speed control
 * - Soft and hard stops
 */

#include <MotorController.h>
using namespace MotorControl;

// Define GPIO pins for TB6612FNG
const int MOTOR_IN1 = 5; // AIN1 - Direction control
const int MOTOR_IN2 = 6; // AIN2 - Direction control
const int MOTOR_PWM = 7; // PWMA - Speed control

// Create motor controller instance
// Parameters: IN1, IN2, PWM, fault_pin, ledc_channel, ledc_timer
MotorController motor(MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, -1, 0, 0);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give time for serial monitor to open

  Serial.println("\nTB6612FNG Motor Controller LEDC Example");

  // Initialize motor controller with 20kHz PWM frequency and 10-bit resolution
  if (!motor.begin(20000, 10)) {
    Serial.println("Failed to initialize motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  Serial.println("Motor controller initialized successfully");
}

void loop() {
  // Forward motion at 50% speed
  Serial.println("Moving forward at 50% speed");
  motor.motorGo(512); // 512/1023 ≈ 50%
  delay(2000);

  // Soft stop (coasting)
  Serial.println("Soft stop (coasting)");
  motor.softStop();
  delay(1000);

  // Backward motion at 30% speed
  Serial.println("Moving backward at 30% speed");
  motor.motorGo(-307); // 307/1023 ≈ 30%
  delay(2000);

  // Hard stop (braking)
  Serial.println("Hard stop (braking)");
  motor.hardStop();
  delay(1000);

  // Speed ramping demonstration
  Serial.println("Demonstrating speed ramping (0-100%)");
  for (int speed = 0; speed <= 1023; speed += 50) {
    motor.motorGo(speed);
    Serial.printf("Speed: %d/1023 (%.1f%%)\n", speed, speed * 100.0 / 1023.0);
    delay(100);
  }

  // Gradually slow down
  Serial.println("Gradually slowing down (100-0%)");
  for (int speed = 1023; speed >= 0; speed -= 50) {
    motor.motorGo(speed);
    Serial.printf("Speed: %d/1023 (%.1f%%)\n", speed, speed * 100.0 / 1023.0);
    delay(100);
  }

  delay(1000);
}