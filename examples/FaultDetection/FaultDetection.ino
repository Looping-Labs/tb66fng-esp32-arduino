/**
 * @file FaultDetection.ino
 * @brief Example of using fault detection with the MotorController library
 *
 * This example demonstrates how to use the fault detection feature to
 * protect your motor and circuits from abnormal conditions like overcurrent
 * or overtemperature that might be reported by the TB6612FNG driver.
 */

#include <MotorController.h>

// Define GPIO pins for TB6612FNG
const int MOTOR_IN1 = 5; // AIN1 - Direction control
const int MOTOR_IN2 = 6; // AIN2 - Direction control
const int MOTOR_PWM = 7; // PWMA - Speed control
const int FAULT_PIN = 8; // Fault signal from motor driver (active low)

// LED pin for fault indication
const int FAULT_LED_PIN = 2;

// Flag to track fault state
volatile bool fault_detected = false;

// Create motor controller instance with fault detection
MotorControl::MotorController motor(MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, FAULT_PIN);

// Fault detection callback
void onFaultDetected() {
  fault_detected = true;
  // Turn on fault LED immediately from ISR
  digitalWrite(FAULT_LED_PIN, HIGH);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give time for serial monitor to open

  // Setup fault indication LED
  pinMode(FAULT_LED_PIN, OUTPUT);
  digitalWrite(FAULT_LED_PIN, LOW);

  Serial.println("\nTB6612FNG Motor Controller Fault Detection Example");

  // Initialize motor controller with 20kHz PWM frequency and 10-bit resolution
  if (!motor.begin(20000, 10)) {
    Serial.println("Failed to initialize motor controller!");
    while (1)
      ; // Don't continue if initialization failed
  }

  // Register fault detection callback
  if (!motor.setFaultDetection(onFaultDetected)) {
    Serial.println("Failed to set up fault detection!");
  } else {
    Serial.println("Fault detection initialized");
  }

  Serial.println("Motor controller ready");
  Serial.println("If a fault occurs (e.g., overcurrent, overtemperature),");
  Serial.println("the LED will turn on and the motor will stop.");
}

void loop() {
  if (fault_detected) {
    // In fault state, attempt recovery
    handleFault();
  } else {
    // Normal operation - run a simple pattern
    runMotorPattern();
  }
}

// Function to handle fault condition
void handleFault() {
  // Ensure motor is stopped
  motor.hardStop();

  // Print fault notification if it's the first time we detect it
  static bool fault_reported = false;

  if (!fault_reported) {
    Serial.println("\n*** FAULT DETECTED! ***");
    Serial.println("Possible causes:");
    Serial.println("- Overcurrent condition");
    Serial.println("- Overtemperature in driver IC");
    Serial.println("- Short circuit");
    Serial.println("- Undervoltage");
    Serial.println("\nAttempting recovery in 5 seconds...");

    fault_reported = true;
  }

  // Wait 5 seconds before attempting recovery
  static unsigned long fault_time = millis();
  if (millis() - fault_time >= 5000) {
    Serial.println("Attempting to clear fault and resume operation...");

    // Reset fault flags
    fault_detected = false;
    fault_reported = false;

    // Turn off fault LED
    digitalWrite(FAULT_LED_PIN, LOW);

    // In a real application, you might check if the fault condition has
    // been resolved before resuming operation
    fault_time = millis();
  }
}

// Function to run a simple motor pattern
void runMotorPattern() {
  // Simple pattern: forward -> stop -> backward -> stop
  static int pattern_state = 0;
  static unsigned long last_change = 0;

  if (millis() - last_change >= 2000) { // Change state every 2 seconds
    pattern_state = (pattern_state + 1) % 4;
    last_change = millis();

    switch (pattern_state) {
    case 0: // Forward at 50% speed
      Serial.println("Forward at 50% speed");
      motor.motorGo(512);
      break;

    case 1: // Soft stop
      Serial.println("Soft stop");
      motor.softStop();
      break;

    case 2: // Backward at 50% speed
      Serial.println("Backward at 50% speed");
      motor.motorGo(-512);
      break;

    case 3: // Hard stop
      Serial.println("Hard stop");
      motor.hardStop();
      break;
    }
  }
}