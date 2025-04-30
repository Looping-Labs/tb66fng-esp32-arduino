/**
 * @file MotorController.cpp
 * @brief Implementation of the MotorController class using LEDC
 */

#include "MotorController.h"

namespace MotorControl {

  // Define the static member variable
  void (*MotorController::fault_callback)(void) = nullptr;

  // ISR handler for fault detection
  void IRAM_ATTR MotorController::handleFaultISR() {
    if (fault_callback) {
      fault_callback();
    }
  }

  // Constructor without standby pin and fault pin
  MotorController::MotorController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin, uint8_t ledc_channel) {
    // Store pins
    this->in1_pin = in1_pin;
    this->in2_pin = in2_pin;
    this->pwm_pin = pwm_pin;
    this->standby_pin = -1; // No standby pin
    this->fault_pin = -1;  // No fault pin
    this->ledc_channel = ledc_channel;
  }

  // Constructor without standby pin
  MotorController::MotorController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin, uint8_t ledc_channel, uint8_t fault_pin) {
    // Store pins
    this->in1_pin = in1_pin;
    this->in2_pin = in2_pin;
    this->pwm_pin = pwm_pin;
    this->standby_pin = -1; // No standby pin
    this->fault_pin = fault_pin;
    this->ledc_channel = ledc_channel;
  }

  // Constructor with standby pin
  MotorController::MotorController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin, uint8_t standby_pin, uint8_t ledc_channel, uint8_t fault_pin) {
    // Store pins
    this->in1_pin = in1_pin;
    this->in2_pin = in2_pin;
    this->pwm_pin = pwm_pin;
    this->standby_pin = standby_pin;
    this->fault_pin = fault_pin;
    this->ledc_channel = ledc_channel;
  }

  bool MotorController::begin(uint32_t freq_hz, uint8_t resolution_bits) {
    // Validate parameters
    if (resolution_bits < 1 || resolution_bits > 20) {
      Serial.println("Invalid resolution bits. Must be between 1 and 20 for ESP32.");
      return false;
    }

    this->resolution_bits = resolution_bits;
    this->max_duty_value = (1 << resolution_bits) - 1;

    // Configure direction pins as outputs
    pinMode(this->in1_pin, OUTPUT);
    pinMode(this->in2_pin, OUTPUT);

    // Initialize direction pins to OFF state
    digitalWrite(this->in1_pin, LOW);
    digitalWrite(this->in2_pin, LOW);

    // Configure standby pin if available
    if (this->standby_pin >= 0) {
      pinMode(this->standby_pin, OUTPUT);
      digitalWrite(this->standby_pin, LOW); // Start in standby mode
      Serial.printf("Standby pin configured on pin %d\n", this->standby_pin);
    }

    // Use the new ledcAttach API that consolidates ledcSetup and ledcAttachPin
    if (ledcAttachChannel(this->pwm_pin, freq_hz, this->resolution_bits, this->ledc_channel)) {
      // Start with 0 duty cycle
      ledcWrite(this->pwm_pin, 0);

      Serial.printf("Motor PWM configured on pin %d with channel %d, %d bits resolution and %u Hz frequency\n", this->pwm_pin, this->ledc_channel, this->resolution_bits, freq_hz);
    } else {
      Serial.printf("Failed to attach LEDC channel %d to PWM pin %d\n", this->ledc_channel, this->pwm_pin);
      return false;
    }

    // Configure fault detection if pin specified
    if (this->fault_pin >= 0) {
      pinMode(this->fault_pin, INPUT_PULLUP);
      Serial.printf("Fault detection initialized on pin %d\n", this->fault_pin);
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
    if (this->fault_pin < 0) {
      Serial.println("Fault pin not configured");
      return false;
    }

    // Store callback
    this->fault_callback = callback;

    // Attach interrupt to fault pin
    attachInterrupt(digitalPinToInterrupt(this->fault_pin), handleFaultISR, FALLING);

    Serial.println("Fault detection ISR configured");
    return true;
  }

  bool MotorController::setDirection(Mode mode) {
    switch (mode) {
    case Mode::FORWARD:
      digitalWrite(this->in1_pin, HIGH);
      digitalWrite(this->in2_pin, LOW);
      break;

    case Mode::BACKWARD:
      digitalWrite(this->in1_pin, LOW);
      digitalWrite(this->in2_pin, HIGH);
      break;

    case Mode::SOFT_STOP:
      digitalWrite(this->in1_pin, LOW);
      digitalWrite(this->in2_pin, LOW);
      break;

    case Mode::HARD_STOP:
      digitalWrite(this->in1_pin, HIGH);
      digitalWrite(this->in2_pin, HIGH);
      break;
    }

    return true;
  }

  bool MotorController::motorGo(int16_t speed) {
    // Convert speed (e.g., -1023 to 1023) to direction and duty cycle
    if (speed > 0) {
      // Forward
      uint16_t duty = min(static_cast<uint16_t>(speed), static_cast<uint16_t>(this->max_duty_value));
      return setMode(Mode::FORWARD, duty);
    } else if (speed < 0) {
      // Backward
      uint16_t duty = min(static_cast<uint16_t>(-speed), static_cast<uint16_t>(this->max_duty_value));
      return setMode(Mode::BACKWARD, duty);
    } else {
      // Stop
      return softStop();
    }
  }

  bool MotorController::setMode(Mode mode, uint16_t duty_cycle) {
    // Cap duty cycle at maximum value based on resolution
    this->current_duty = min(duty_cycle, static_cast<uint16_t>(this->max_duty_value));
    this->current_mode = mode;

    // Set direction pins
    if (!setDirection(mode)) {
      return false;
    }

    // Set PWM duty cycle directly to pin (new API approach)
    ledcWrite(this->pwm_pin, this->current_duty);

    return true;
  }

  bool MotorController::softStop() {
    return setMode(Mode::SOFT_STOP, 0);
  }

  bool MotorController::hardStop() {
    return setMode(Mode::HARD_STOP, 0);
  }

  bool MotorController::enable() {
    if (this->standby_pin < 0) {
      Serial.println("Standby pin not configured");
      return false;
    }

    digitalWrite(this->standby_pin, HIGH);
    Serial.println("Motor driver enabled");
    return true;
  }

  bool MotorController::disable() {
    if (this->standby_pin < 0) {
      Serial.println("Standby pin not configured");
      return false;
    }

    digitalWrite(this->standby_pin, LOW);
    Serial.println("Motor driver disabled (standby mode)");
    return true;
  }

  void MotorController::end() {
    // Turn off PWM before detaching
    ledcWrite(this->pwm_pin, 0);

    // Detach LEDC using new API name
    ledcDetach(this->pwm_pin);

    // Reset pins to input to avoid any unwanted signals
    pinMode(this->in1_pin, INPUT);
    pinMode(this->in2_pin, INPUT);
    pinMode(this->pwm_pin, INPUT);

    // Set standby pin LOW if configured
    if (this->standby_pin >= 0) {
      digitalWrite(this->standby_pin, LOW);
      pinMode(this->standby_pin, INPUT);
    }

    // Detach interrupt if fault pin was configured
    if (this->fault_pin >= 0) {
      detachInterrupt(digitalPinToInterrupt(this->fault_pin));
      pinMode(this->fault_pin, INPUT);
    }
  }
} // namespace MotorControl