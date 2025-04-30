/**
 * @file MotorController.h
 * @brief Motor controller library for TB6612FNG driver with ESP32 LEDC
 *
 * This library provides motor control capabilities using ESP32's LEDC
 * peripheral for PWM generation, designed specifically for the TB6612FNG
 * motor driver. Updated for Arduino ESP32 3.0+ compatibility.
 *
 * @author json-dev
 * @date April 2025
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

namespace MotorControl {

  /**
   * @class MotorController
   * @brief TB6612FNG motor controller using ESP32 LEDC
   *
   * This class provides a simple interface for controlling DC motors
   * using the TB6612FNG motor driver with ESP32's LEDC peripheral
   * for PWM generation. Compatible with Arduino ESP32 3.0+.
   */
  class MotorController {
  public:
    /**
     * @brief Operating modes for the motor
     */
    enum class Mode {
      FORWARD,   ///< Motor rotating forward
      BACKWARD,  ///< Motor rotating backward
      SOFT_STOP, ///< Motor coasting to stop (both pins LOW)
      HARD_STOP  ///< Motor brake (both pins HIGH)
    };

    /**
     * @brief Constructor for Motor Controller (without standby pin)
     *
     * @param in1_pin Direction control pin 1
     * @param in2_pin Direction control pin 2
     * @param pwm_pin PWM speed control pin
     * @param fault_pin Optional GPIO for fault detection (set to -1 if not used)
     * @param ledc_channel LEDC channel to use (0-15 on ESP32)
     */
    MotorController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin, uint8_t ledc_channel, uint8_t fault_pin);

    /**
     * @brief Constructor for Motor Controller (without standby pin)
     *
     * @param in1_pin Direction control pin 1
     * @param in2_pin Direction control pin 2
     * @param pwm_pin PWM speed control pin
     * @param fault_pin Optional GPIO for fault detection (set to -1 if not used)
     * @param ledc_channel LEDC channel to use (0-15 on ESP32)
     */
    MotorController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin, uint8_t ledc_channel);

    /**
     * @brief Constructor for Motor Controller (with standby pin)
     *
     * @param in1_pin Direction control pin 1
     * @param in2_pin Direction control pin 2
     * @param pwm_pin PWM speed control pin
     * @param standby_pin Standby control pin
     * @param fault_pin Optional GPIO for fault detection (set to -1 if not used)
     * @param ledc_channel LEDC channel to use (0-15 on ESP32)
     */
    MotorController(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin, uint8_t standby_pin, uint8_t ledc_channel, uint8_t fault_pin);

    /**
     * @brief Initialize the motor controller
     *
     * @param freq_hz PWM frequency in Hz (default 10kHz)
     * @param resolution_bits PWM resolution in bits (1-20 for ESP32, default 10)
     * @return bool true on success, false on failure
     */
    bool begin(uint32_t freq_hz = 10000, uint8_t resolution_bits = 10);

    /**
     * @brief Start motor rotation with specified speed and direction
     *
     * @param speed Speed value from -max to max, sign determines direction
     * @return bool true on success, false on failure
     *
     * @note Actual speed range depends on the resolution set in begin().
     *       With 10-bit resolution, range is -1023 to 1023.
     */
    bool motorGo(int16_t speed);

    /**
     * @brief Set motor to specific mode and duty cycle
     *
     * @param mode Operation mode (FORWARD, BACKWARD, SOFT_STOP, HARD_STOP)
     * @param duty_cycle Duty cycle from 0 to max value based on resolution
     * @return bool true on success, false on failure
     */
    bool setMode(Mode mode, uint16_t duty_cycle = 0);

    /**
     * @brief Stop motor softly (coast to stop)
     * @return bool true on success, false on failure
     */
    bool softStop();

    /**
     * @brief Stop motor hard (brake)
     * @return bool true on success, false on failure
     */
    bool hardStop();

    /**
     * @brief Set up fault detection ISR
     *
     * @param callback Function to call when fault is detected
     * @return bool true on success, false on failure
     */
    bool setFaultDetection(void (*callback)(void));

    /**
     * @brief Enable motor driver by setting standby pin HIGH
     * @return bool true on success, false if standby pin not configured
     */
    bool enable();

    /**
     * @brief Disable motor driver by setting standby pin LOW
     * @return bool true on success, false if standby pin not configured
     */
    bool disable();

    /**
     * @brief Check if standby pin is configured
     * @return bool true if standby pin is configured, false otherwise
     */
    bool hasStandbyPin() const { return standby_pin >= 0; }

    /**
     * @brief Get the currently assigned LEDC channel
     * @return uint8_t The LEDC channel number
     */
    uint8_t getLedcChannel() const { return ledc_channel; }

    /**
     * @brief Clean up resources and detach LEDC
     */
    void end();

  private:
    // GPIO pins
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t pwm_pin;
    uint8_t standby_pin;
    uint8_t fault_pin;

    // LEDC configuration
    uint8_t  ledc_channel; 
    uint8_t  resolution_bits;
    uint32_t max_duty_value;

    // Current operating state
    Mode current_mode = Mode::SOFT_STOP;
    uint16_t current_duty = 0;

    // Configure direction pins
    bool setDirection(Mode mode);

    // Static fault callback pointer
    static void (*fault_callback)(void);

    // Static ISR handler
    static void IRAM_ATTR handleFaultISR();
  };
} // namespace MotorControl

#endif // MOTOR_CONTROLLER_H