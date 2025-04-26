/**
 * @file MotorController.cpp
 * @brief Implementation of the MotorController class using LEDC
 */

#include "MotorController.h"
#include "driver/ledc.h"

namespace MotorControl {

  // ISR handler for fault detection
  void IRAM_ATTR MotorController::handleFaultISR() {
    if (fault_callback_) {
      fault_callback_();
    }
  }

  MotorController::MotorController(int in1_pin,
                                   int in2_pin,
                                   int pwm_pin,
                                   int fault_pin) {
    // Store pins
    in1_pin_ = in1_pin;
    in2_pin_ = in2_pin;
    pwm_pin_ = pwm_pin;
    fault_pin_ = fault_pin;
  }

  bool MotorController::begin(uint32_t freq_hz, uint8_t resolution_bits) {
    // Validate parameters
    if (resolution_bits < 8 || resolution_bits > 14) {
      Serial.println("Invalid resolution bits. Must be between 8 and 14.");
      return false;
    }

    resolution_bits_ = resolution_bits;
    max_duty_value_ = (1 << resolution_bits) - 1;

    // Configure direction pins as outputs
    pinMode(in1_pin_, OUTPUT);
    pinMode(in2_pin_, OUTPUT);

    // Initialize direction pins to OFF state
    digitalWrite(in1_pin_, LOW);
    digitalWrite(in2_pin_, LOW);

    // Configure LEDC for PWM using the new ledcAttach API
    // This replaces the previous ledcSetup and ledcAttachPin calls
    if (ledcAttach(pwm_pin_, freq_hz, resolution_bits_) == ESP_OK) {
      // Start with 0 duty cycle
      ledcWrite(pwm_pin_, 0);

      Serial.printf("Motor PWM configured on pin %d with %d bits resolution and %u Hz frequency\n",
                    pwm_pin_, resolution_bits_, freq_hz);
    } else {
      Serial.printf("Failed to attach LEDC to PWM pin %d\n", pwm_pin_);
      return false;
    }

    // Configure fault detection if pin specified
    if (fault_pin_ >= 0) {
      pinMode(fault_pin_, INPUT_PULLUP);
      Serial.printf("Fault detection initialized on pin %d\n", fault_pin_);
    }

    // Set initial state to SOFT_STOP
    if (!setMode(Mode::SOFT_STOP)) {
      Serial.println("Failed to set initial mode");
      return false;
    }

    Serial.println("Motor controller initialized successfully");
    return true;
  }

  bool MotorController::setFaultDetection(void (*callback)(void)) {
    if (fault_pin_ < 0) {
      Serial.println("Fault pin not configured");
      return false;
    }

    // Store callback
    fault_callback_ = callback;

    // Attach interrupt to fault pin
    attachInterrupt(digitalPinToInterrupt(fault_pin_), handleFaultISR, FALLING);

    Serial.println("Fault detection ISR configured");
    return true;
  }

  bool MotorController::setDirection(Mode mode) {
    switch (mode) {
    case Mode::FORWARD:
      digitalWrite(in1_pin_, HIGH);
      digitalWrite(in2_pin_, LOW);
      break;

    case Mode::BACKWARD:
      digitalWrite(in1_pin_, LOW);
      digitalWrite(in2_pin_, HIGH);
      break;

    case Mode::SOFT_STOP:
      digitalWrite(in1_pin_, LOW);
      digitalWrite(in2_pin_, LOW);
      break;

    case Mode::HARD_STOP:
      digitalWrite(in1_pin_, HIGH);
      digitalWrite(in2_pin_, HIGH);
      break;
    }

    return true;
  }

  bool MotorController::motorGo(int16_t speed) {
    // Convert speed (e.g., -1023 to 1023) to direction and duty cycle
    if (speed > 0) {
      // Forward
      uint16_t duty = min(static_cast<uint16_t>(speed), static_cast<uint16_t>(max_duty_value_));
      return setMode(Mode::FORWARD, duty);
    } else if (speed < 0) {
      // Backward
      uint16_t duty = min(static_cast<uint16_t>(-speed), static_cast<uint16_t>(max_duty_value_));
      return setMode(Mode::BACKWARD, duty);
    } else {
      // Stop
      return softStop();
    }
  }

  bool MotorController::setMode(Mode mode, uint16_t duty_cycle) {
    // Cap duty cycle at maximum value based on resolution
    current_duty_ = min(duty_cycle, static_cast<uint16_t>(max_duty_value_));
    current_mode_ = mode;

    // Set direction pins
    if (!setDirection(mode)) {
      return false;
    }

    // Set PWM duty cycle - now using pin instead of channel
    ledcWrite(pwm_pin_, current_duty_);

    return true;
  }

  bool MotorController::softStop() {
    return setMode(Mode::SOFT_STOP, 0);
  }

  bool MotorController::hardStop() {
    return setMode(Mode::HARD_STOP, 0);
  }

  void MotorController::end() {
    // Use the updated ledcDetach API (replacement for ledcDetachPin)
    ledcDetach(pwm_pin_);

    // Reset pins to input to avoid any unwanted signals
    pinMode(in1_pin_, INPUT);
    pinMode(in2_pin_, INPUT);
    pinMode(pwm_pin_, INPUT);

    // Detach interrupt if fault pin was configured
    if (fault_pin_ >= 0) {
      detachInterrupt(digitalPinToInterrupt(fault_pin_));
      pinMode(fault_pin_, INPUT);
    }
  }

} // namespace MotorControl