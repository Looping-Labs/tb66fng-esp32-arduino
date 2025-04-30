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

// Define motor control pins
#define R_MOTOR_IN1 19 // TB6612FNG IN1 pin
#define R_MOTOR_IN2 18 // TB6612FNG IN2 pin
#define R_MOTOR_PWM 5  // TB6612FNG PWM input pin (PWMA or PWMB)
#define L_MOTOR_IN1 17 // TB6612FNG IN1 pin
#define L_MOTOR_IN2 16 // TB6612FNG IN2 pin
#define L_MOTOR_PWM 4  // TB6612FNG PWM input pin (PWMA or PWMB)

const uint16_t FREQUENCY = 10000;   // 10kHz
const uint8_t RESOLUTION = 10;      // 10-bit resolution (0-1023)
const uint8_t CHANNEL = 0;          // LEDC channel (0-15)

// Create motor controller instances
MotorController r_motor(R_MOTOR_IN1, R_MOTOR_IN2, R_MOTOR_PWM, CHANNEL);
MotorController l_motor(L_MOTOR_IN1, L_MOTOR_IN2, L_MOTOR_PWM, CHANNEL);

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor to open

  if (!r_motor.begin(FREQUENCY, RESOLUTION)) {
    Serial.println("Failed to initialize right motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  if (!l_motor.begin(FREQUENCY, RESOLUTION)) {
    Serial.println("Failed to initialize left motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  Serial.println("Motors controller initialized successfully");
}

void loop() {
  // Forward motion at 50% speed
  Serial.println("Moving forward at 50% speed");
  r_motor.motorGo(512); // 512/1023 ≈ 50%
  l_motor.motorGo(512); // 512/1023 ≈ 50%
  delay(4000);

  // Soft stop (coasting)
  Serial.println("Soft stop (coasting)");
  r_motor.softStop();
  l_motor.softStop();
  delay(2000);

  // Backward motion at 30% speed
  Serial.println("Moving backward at 30% speed");
  r_motor.motorGo(-307); // 307/1023 ≈ 30%
  l_motor.motorGo(-307); // 307/1023 ≈ 30%
  delay(4000);

  // Hard stop (braking)
  Serial.println("Hard stop (braking)");
  r_motor.hardStop();
  l_motor.hardStop();
  delay(2000);

  // Speed ramping demonstration
  Serial.println("Demonstrating speed ramping (0-100%)");
  for (int speed = 0; speed <= 1023; speed += 50) {
    r_motor.motorGo(speed);
    l_motor.motorGo(speed);
    Serial.printf("Speed: %d/1023 (%.1f%%)\n", speed, speed * 100.0 / 1023.0);
    delay(100);
  }

  delay(2000);

  // Gradually slow down
  Serial.println("Gradually slowing down (100-0%)");
  for (int speed = 1023; speed >= 0; speed -= 50) {
    r_motor.motorGo(speed);
    l_motor.motorGo(speed);
    Serial.printf("Speed: %d/1023 (%.1f%%)\n", speed, speed * 100.0 / 1023.0);
    delay(100);
  }

  delay(1000);
}