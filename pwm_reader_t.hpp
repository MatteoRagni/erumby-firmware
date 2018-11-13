#ifndef INTERRUPT_MANAGER_HPP
#define INTERRUPT_MANAGER_HPP

/**
 * \file interrupt_manager.hpp
 * \author Matteo Ragni
 *
 * This file contains two helper classes that allows to easily read the
 * pwm signal on a pin. To to some peculiarities this class is taylored
 * made for the Arduino Mega 2560 which is on the erumby car, but it
 * is quite easy to generalize it for different boards, using the same
 * approach.
 *
 * There are two classes of pins which are considered:
 *  - pins for which the \p attachInterrupt function can be used, which
 *    are handled by \p pwm_reader_attachable_t and have a digital read
 *  - pins for which we must configure the registers to intercept the
 *    interrupts, which are handled by \p pwm_reader_t
 *
 * The implementation is made in a way that for the user the declaration
 * of a pwm input pin as simple as possible. For example:
 * @code
 * pwm_reader_t pwm = pwm_reader_t(52); // Attach to pin 52
 *
 * // to get the current read
 * pwm.get_pulse();
 * @endcode
 *
 * For the specific board considered:
 *  - attachable pins: 2, 3, 18, 19, 20, 21 (no more can be added)
 *  - non-attachable pin: 52, 53, A8, A9 (more can be implemented)
 *
 * The non attachable interrupt are particularly troublesome, because
 * it is necessary to set up the registers. For each pin it is necessary
 * to know:
 *  - the belonging port (e.g. 52 and 53 are in PORT_B, A8 and A9 in PORT_K)
 *  - the position on the port (also known as mask, a 8-bit value)
 *
 * The algorithm for reading the pulse is quite simple, and it is performed
 * in an interrupt routine:
 * @code
 *  read the value of the pin
 *  if the pin is high
 *    store the current raise time
 *  else
 *    evaluate the pulse time
 *  @endcode
 * The classes have also a counter and a stabilized pulse value for some
 * specific applications.
 */

#include <Arduino.h>
#include "configurations.hpp"
#include "types.hpp"

#define DUTY_MODE_DELTA 500                      /**< A delta for reading the mode (erumby remote specific) */
#define DUTY_MODE_STABILIZER 10                  /**< Number of read for stabilization */
typedef void (*pwm_attachable_callback_t)(void); /**< Callback type for attachable pins */
typedef pin_t (*pwm_port_reader_t)(void);

/** \brief Class for reading non attachable pins
 *
 * Looking at the Arduino documentation there are only few pins
 * which are interrupt capable. This class allows to use pins that are not
 * in that list to read pwms, even if this requires some additonal informations
 *
 * The class reads the pwm on some interrupt cabable, non attachable pin.
 * This means that we need to handle the register configuration to make the
 * microcontroller to check for interrupts. Three informations are rquired for
 * the setup:
 *  - number of pin
 *  - which port the pin belongs (each port handles 8 pins)
 *  - map of the pin on the port (position in the port of the pin)
 * this information must be hardcoded in the class. As for now the information
 * have been coded only for the following pins:
 *
 * | Pin | Port              | Map            |
 * |-----|-------------------|----------------|
 * | 52  | `B (PCINT0_vect)` | `0b 0000 0010` |
 * | 53  | `B (PCINT0_vect)` | `0b 0000 0001` |
 * | A8  | `K (PCINT2_vect)` | `0b 0000 0001` |
 * | A9  | `K (PCINT2_vect)` | `0b 0000 0010` |
 *
 * For each port there is only 1 interrupt routine, thus the interrupt callback
 * handles if we have a modification on our pin or not.
 *
 * \warning Do not inherit from this class!
 * \warning If the pin connected does not belong to the table the
 * sketch blocks the execution showing an error, by pulsing the led of the board.
 * The error code is a loop of `[ + + + + - + - + - ]`:
 *  - On for 2s,
 *  - Off for 0.5s,
 *  - On for 0.5s,
 *  - Off for 0.5s,
 *  - On for 0.5s,
 *  - Off for 0.5s
 */
class pwm_reader_t {
  pin_t pin;         /**< physical pin on the board */
  pin_t map;         /**< position of the pin in the port */
  pulse_t edge_time; /**< timing of the last high edge */
  pin_t read;        /**< last pin reading (high/low) */
  pulse_t pulse;     /**< duration of the high edge (the pwm reading) */
  counter_t counter; /**< counter (actually used for the encoder) */
  pwm_port_reader_t port_reader; /**< reader for the specific pin */

 public:
  static pwm_reader_t* portB[8]; /**< pointer to pin with pwm on port B. Never manually edit this value. */
  static pwm_reader_t* portK[8]; /**< pointer to pin with pwm on port K. Never manually edit this value. */
  static pin_t portB_count;      /**< number of pin connected on port B. Never manually edit this value. */
  static pin_t portK_count;      /**< number of pin connected on port K. Never manually edit this value. */

  /** \brief Constructor for the pwm reader
   *
   * The constructor uses some of the static functions in order to get
   * port and map and register the interrut callback for this pin to the
   * overall callback of the port.
   * It may raise an alarm if the pin is not included in the following table:
   *
   * | Pin | Port              | Map            |
   * |-----|-------------------|----------------|
   * | 52  | `B (PCINT0_vect)` | `0b 0000 0010` |
   * | 53  | `B (PCINT0_vect)` | `0b 0000 0001` |
   * | A8  | `K (PCINT2_vect)` | `0b 0000 0001` |
   * | A9  | `K (PCINT2_vect)` | `0b 0000 0010` |
   *
   * \param pin the pin for reading the pwm
   */
  pwm_reader_t(pin_t pin_) : pin(pin_), pulse(0), counter(0), read(0), edge_time(0) {
    map = pwm_reader_t::pin_map(pin);
    port_reader = pwm_reader_t::get_port_reader(pin);
    pwm_reader_t::init_interrupt(this);
  }

  /** \brief Interrupt callback registered at pin creation
   *
   * This interrupt callback is registered for execution at object
   * construction. The callback reads the status of the pin and evaluates
   * the duration of the pwm pulse.
   * In this case, since it is used by the encoder, there is also some
   * counter handling.
   */
  void interrupt_callback();

  /** \brief Get current counter value */
  inline const counter_t get_counter() { return counter; }
  /** \brief Resets the counter to 0 */
  inline void reset_counter() { counter = 0; }
  /** \brief Get the current pulse reading */
  inline const pulse_t get_pulse() { return pulse; }

  /** \brief The actual interrupt service routine for port B
   *
   * \warning Never use directly this function.
   *
   * The registration of the callback shall be done as follows:
   *
   * @code
   * ISR(PCINT0_vect, ISR_NOBLOCK) {
   *   pwm_reader_t::portB_isr();
   * }
   * @endcode
   *
   * this is included in the implementation file \p pwm_reader_t.cpp
   */
  static void portB_isr();
  /** \brief The actual interrupt service routine for port K
   *
   * \warning Never use directly this function.
   *
   * The registration of the callback shall be done as follows:
   *
   * @code
   * ISR(PCINT2_vect, ISR_NOBLOCK) {
   *   pwm_reader_t::portK_isr();
   * }
   * @endcode
   *
   * this is included in the implementation file \p pwm_reader_t.cpp
   */
  static void portK_isr();
  
 private:
  /** \biref Inits the interrupts on port B
   *
   * The function sets the pin as input with pullup, then
   * enables in the register \p PCMSK0 (pin change mask register
   * for port B), and enables the port in the PCICR (pin group interrupt
   * register).
   * \warning Never use directly this function
   * 
   * \param pwm current \p pwm_reader_t obect pointer (\p this) 
   */
  static void init_port_B(pwm_reader_t* pwm);
  /** \biref Inits the interrupts on port B
   *
   * The function sets the pin as input with pullup, then
   * enables in the register \p PCMSK2 (pin change mask register
   * for port K), and enables the port in the PCICR (pin group interrupt
   * register).
   * \warning Never use directly this function
   * 
   * \param pwm current \p pwm_reader_t obect pointer (\p this) 
   */
  static void init_port_K(pwm_reader_t* pwm);

  /** \brief Returns the map for a specific pin
   * 
   * The function contains the map for each pin, where the map
   * is the position of the pin in the port.
   * 
   * \param pin the pin for which the map is required
   * \return the map for the pin 
   */
  static pin_t pin_map(pin_t pin);

  /** \brief Initialize the interrupt port
   * 
   * Calls the correct port to initialize in function 
   * of the pin and the map.
   * 
   * \param pwm current \p pwm_reader_t obect pointer (\p this)
   */
  static void init_interrupt(pwm_reader_t* pwm);

  /** \brief Returns the reading of the pin
   * 
   * Returns the reader for the pin of the port. This function
   * returns a lambda with the procedure for reading the port.
   * 
   * \param pin the pin to read
   * \return the lambda that reads the port
   */
  static  pwm_port_reader_t get_port_reader(pin_t pin);

 public:
  /** \brief Alarm function */
  static void interrupt_error();
};

class pwm_reader_attachable_t {
  pin_t pin;
  pulse_t pulse;
  pulse_t pulse_real;
  pulse_t edge_time;
  counter_t counter;
  pin_t read;

 public:
  static uint8_t pin_counter;
  static pwm_reader_attachable_t* pin_table[6];
  static const pwm_attachable_callback_t callbacks[6];

  pwm_reader_attachable_t(pin_t pin_) : pin(pin_) {
    counter = 0;
    pulse = 0;
    pulse_real = 0;
    edge_time = 0;
    read = 0;
    pwm_reader_attachable_t::register_callback(this, pin);
  }

  inline counter_t get_counter() { return counter; }
  inline pulse_t get_pulse() { return pulse; }
  inline pulse_t get_pulse_real() { return pulse_real; }

  void interrupt_callback() {
    uint16_t c_time = micros();

    if (digitalRead(pin))
      edge_time = c_time;
    else
      pulse = c_time - edge_time;

    if (abs(pulse_real - pulse) > DUTY_MODE_DELTA)
      counter++;
    else
      counter = 0;

    if (counter == DUTY_MODE_STABILIZER) {
      pulse_real = pulse;
      counter = 0;
    }
  }

 private:
  static void register_callback(pwm_reader_attachable_t* self, pin_t pin) {
    switch (pin) {
      case (2):
      case (3):
      case (18):
      case (19):
      case (20):
      case (21):
        pwm_reader_attachable_t::pin_table[pwm_reader_attachable_t::pin_counter] = self;
        attachInterrupt(digitalPinToInterrupt(pin),
                        pwm_reader_attachable_t::callbacks[pwm_reader_attachable_t::pin_counter], CHANGE);
        pwm_reader_attachable_t::pin_counter++;
        break;
      default:
        pwm_reader_attachable_t::interrupt_error();
        break;
    }
  }

  static void interrupt_error() {
    pinMode(ERROR_LED_PORT, OUTPUT);
    digitalWrite(ERROR_LED_PORT, LOW);
    while (1) {
      digitalWrite(ERROR_LED_PORT, HIGH);
      delay(2000);
      digitalWrite(ERROR_LED_PORT, LOW);
      delay(500);
      digitalWrite(ERROR_LED_PORT, HIGH);
      delay(500);
      digitalWrite(ERROR_LED_PORT, LOW);
      delay(500);
      digitalWrite(ERROR_LED_PORT, HIGH);
      delay(500);
      digitalWrite(ERROR_LED_PORT, LOW);
      delay(500);
    }
  }
};

/* void pwm_reader_mode_callback();

class pwm_reader_mode_t {
  pin_t pin;
  pulse_t pulse;
  pulse_t pulse_real;
  pulse_t edge_time;
  counter_t counter;
  pin_t read;

 public:
  static pwm_reader_mode_t* self;
  pwm_reader_mode_t(pin_t pin_) : pin(pin_) {
    if (self)
      return;
    self = this;
    counter = 0;
    pulse = 0;
    pulse_real = 0;
    edge_time = 0;
    read = 0;

    attachInterrupt(digitalPinToInterrupt(pin), pwm_reader_mode_callback, CHANGE);
  }

  inline counter_t get_counter() { return counter; }
  inline pulse_t get_pulse() { return pulse; }
  inline pulse_t get_pulse_real() { return pulse_real; }

  void interrupt_callback() {
    uint16_t c_time = micros();

    if (digitalRead(pin))
      edge_time = c_time;
    else
      pulse = c_time - edge_time;

    if (abs(pulse_real - pulse) > DUTY_MODE_DELTA)
      counter++;
    else
      counter = 0;

    if (counter == 10) {
      pulse_real = pulse;
      counter = 0;
    }
  }
}; */

#endif /* INTERRUPT_MANAGER_HPP */