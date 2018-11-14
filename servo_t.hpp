#ifndef SERVO_T_HPP
#define SERVO_T_HPP

/**
 * \file servo_t.hpp
 * \author Davide Piscini, Matteo Ragni
 *
 * ** Class for the SERVO (Servo motor controller)**
 *
 * The class implements the software representation of the
 * Servo motor Controller of the car. It takes a
 * PWM as input and sets a voltage on the servo input
 * in order to get movement. The SERVO class also implements
 * some security features such raising an alarm when the input
 * is out the boundaries.
 *
 * The user should be aware that for the current hardware the
 * full right control is the minimum value in terms of PWM
 * while the full left control is the maximum value in terms of PWM.
 * The code handles the opposite situation at compile time
 */

#include "configurations.hpp"
#include "types.hpp"

/** \brief Class for the SERVO (Servo motor controller)
 *
 * The class implements the software representation of the
 * Servo motor Controller of the car. It takes a
 * PWM as input and sets a voltage on the motor input
 * in order to get movement. The SERVO class also implements
 * some security features such raising an alarm when the input
 * is out the boundaries.
 */
class servo_t {
  const pin_t pin;    /**< Pin for the SERVO, this is specified in \p configurations.hpp */
  cmd_t value;        /**< PWM value that is currently on PWM */
  cmd_t queued_value; /**< PWM requested by the user */
  erumby_base_t* m; /**< Pointer to erumby main instance */

  /** \brief Check the user input. If in remote may raise the alarm
   *
   * The function limits the input between the boundaries of the duty
   * cycle of the servo (\p DUTY_SERVO_DX, \p DUTY_SERVO_SX). If the
   * machine is in remote mode, then the \p erumby_t::alarm will be
   * called for an input out of bounds.
   *
   * In the initial case, the full right control is considered to be
   * the smaller PWM value possible, while the full left is considered
   * the full right. In case of the opposite situation, the code
   * fix the boundaries automatically, at compile time.
   *
   * \param v the user input to be checked
   * \return the value of PWM that will be set
   * \see erumby_alarm
   * \see erumby_mode
   */
  inline cmd_t input_check(cmd_t v) {
    if ((v > get_max()) || (v < get_min())) {
      if (m->mode() == Auto) {
        m->alarm("SERVO");
      }
      return get_center();
    }
    return v;
  }

 public:
  /** \brief Constructor for the servo object
   *
   * The servo object initializes the value for the SERVO PWM
   * and sets the erumby instance pointer. The constructor also
   * initializes the frequency of the PWM pin for the servo.
   * In order to set the frequency, the timers should be initialized
   * by the \p e_rumby constructor.
   *
   * At the end of the constructor the \p stop is called in order to be sure
   * to write immediately on the PWM the idle values.
   *
   * \param m_ pointers to the unique instance of the erumby machine
   */
  servo_t(erumby_base_t* m_) : m(m_), value(DUTY_SERVO_MIDDLE), queued_value(DUTY_SERVO_MIDDLE), pin(SERVO) {
    SetPinFrequency(pin, PWM_FREQUENCY);
    stop();
  };

  /** \brief Puts a queued value for PWM in the class
   *
   * The method allows the user to request a new value for the
   * PWM. The value will be written to the motor once the servo executes
   * a loop. This means that the writes on SERVO is queued to the next
   * iteration.
   * To be clear:
   *  1. the user calls \p set with a value of PWM, the value is in the queue
   *  2. the machine continues with its loop, calling all devices loop
   *  3. When the servo loop is called, if the queued value is different
   *   from the current value, the queued value is written on PWM.
   * To be noted: if the user requires two value **before** at least one
   * loop is executed, the first value will be never exported to the
   * PWM pin (it means that the two request are faster than a single machine loop).
   *
   * This allows to send the command even in an Interrupt Service Routine,
   * or in a communication routine keeping the code execution deterministic.
   *
   * \param v the user value to be queued
   */
  void set(cmd_t v) { queued_value = input_check(v); };

  /** \brief SERVO main loop of execution
   *
   * This function must be called inside the \p erumby_t::loop and
   * it is the main loop for the SERVO. When value is different than
   * the queued value, the queud value is written to the SERVO and stored
   * as the current value.
   *
   * \see set
   */
  void loop() {
    if (value != queued_value) {
      value = queued_value;
      pwmWriteHR(pin, value);
    }
  }

  /** Normal stop
   *
   * The normal stop puts the idle value in the queue in order to
   * arrest the steer n neutral immediately.
   */
  void stop() {
    value = get_center();
    queued_value = get_center();
    pwmWriteHR(pin, value);
  }

  /** 
   * \brief Returns the value currently on the PWM pin
   * \return the value that is currently wirtten in pwm 
   */
  inline const cmd_t get() const { return value; }

  /** 
   * \brief Returns minimum PWM value possible 
   * \return the minim value for the SERVO that it is possible to write
   * \see DUTY_SERVO_DX 
   */
  inline const cmd_t get_full_dx() const { return DUTY_SERVO_DX; }
  
  /** 
   * \brief Returns maximum PWM value possible 
   * \return the minim value for the SERVO that it is possible to write
   * \see DUTY_SERVO_SX 
   */  
  inline const cmd_t get_full_sx() const { return DUTY_SERVO_SX; }

  /** 
   * \brief Returns the idle PWM value 
   * \return the PWM value that stops the motor
   * \see DUTY_SERVO_IDLE
   */
  inline const cmd_t get_center() const { return DUTY_SERVO_MIDDLE; }

#if DUTY_SERVO_SX > DUTY_SERVO_DX
 /** 
  * \brief Returns the maximum value of the PWM for the Servo
  * \return the maximum value for the PWM
  * \see DUTY_SERVO_SX
  */
  inline const cmd_t get_max() { return DUTY_SERVO_SX; }
  /** 
  * \brief Returns the minimum value of the PWM for the Servo
  * \return the minimum value for the PWM
  * \see DUTY_SERVO_DX
  */
  inline const cmd_t get_min() { return DUTY_SERVO_DX; }
#else
  inline const cmd_t get_max() { return DUTY_SERVO_DX; }
  inline const cmd_t get_min() { return DUTY_SERVO_SX; }
#endif
};

#endif /* ESC_T_HPP */