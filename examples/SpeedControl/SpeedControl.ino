/**
 * @file SpeedControl.ino
 * @brief Example of open-loop speed control with the MotorController library
 *
 * This example demonstrates how to create a simple speed control system
 * that allows for precise setting of motor speed with ramping for smooth
 * acceleration and deceleration.
 */

#include <MotorController.h>
#include <algorithm>

using namespace MotorControl;

// Define GPIO pins for TB6612FNG
const int MOTOR_IN1 = 5; // AIN1 - Direction control
const int MOTOR_IN2 = 6; // AIN2 - Direction control
const int MOTOR_PWM = 7; // PWMA - Speed control

// Create motor controller instance
MotorController motor(MOTOR_IN1, MOTOR_IN2, MOTOR_PWM);

// Speed control variables
int16_t target_speed = 0;                 // Target speed (-1023 to 1023)
int16_t current_speed = 0;                // Current speed (-1023 to 1023)
int16_t acceleration = 5;                 // Speed change per step (adjust as needed)
unsigned long speed_update_interval = 20; // Time in ms between speed updates
unsigned long last_speed_update = 0;

// Serial command variables
String command = "";
bool command_complete = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give time for serial monitor to open

  Serial.println("\nTB6612FNG Speed Control Example");
  Serial.println("Commands:");
  Serial.println("  S<value>  - Set target speed (-1023 to 1023)");
  Serial.println("  A<value>  - Set acceleration (1-100)");
  Serial.println("  H         - Hard stop");
  Serial.println("  ?         - Show current settings");

  // Initialize motor controller with 20kHz PWM frequency and 10-bit resolution
  if (!motor.begin(20000, 10)) {
    Serial.println("Failed to initialize motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  Serial.println("Motor controller initialized successfully");
}

void loop() {
  // Check for serial commands
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (command.length() > 0) {
        command_complete = true;
      }
    } else {
      command += c;
    }
  }

  // Process completed command
  if (command_complete) {
    processCommand();
    command = "";
    command_complete = false;
  }

  // Update motor speed with ramping
  unsigned long current_time = millis();
  if (current_time - last_speed_update >= speed_update_interval) {
    updateSpeed();
    last_speed_update = current_time;
  }
}

// Process serial commands
void processCommand() {
  if (command.length() < 1)
    return;

  char cmd = command.charAt(0);
  command.remove(0, 1); // Remove the command character

  switch (cmd) {
  case 'S':
  case 's':
    // Set target speed
    target_speed = constrain(command.toInt(), -1023, 1023);
    Serial.printf("Target speed set to %d\n", target_speed);
    break;

  case 'A':
  case 'a':
    // Set acceleration
    acceleration = constrain(command.toInt(), 1, 100);
    Serial.printf("Acceleration set to %d\n", acceleration);
    break;

  case 'H':
  case 'h':
    // Hard stop
    target_speed = 0;
    current_speed = 0;
    motor.hardStop();
    Serial.println("Hard stop executed");
    break;

  case '?':
    // Show current settings
    Serial.printf("Current speed: %d, Target speed: %d, Acceleration: %d\n",
                  current_speed, target_speed, acceleration);
    break;

  default:
    Serial.println("Unknown command");
    break;
  }
}

// Update motor speed with ramping
void updateSpeed() {
  if (current_speed < target_speed) {
    // Accelerate forward
    current_speed = min(static_cast<int16_t>(current_speed + acceleration), target_speed);
  } else if (current_speed > target_speed) {
    // Decelerate or accelerate backward
    current_speed = max(static_cast<int16_t>(current_speed - acceleration), target_speed);
  }

  // Update motor speed
  if (current_speed != 0) {
    motor.motorGo(current_speed);
  } else if (abs(current_speed - target_speed) < acceleration) {
    // We've reached zero, soft stop
    motor.softStop();
  }
}