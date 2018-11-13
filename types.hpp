#ifndef TYPES_HPP
#define TYPES_HPP

/**
 * \file types.hpp
 * \author Matteo Ragni
 * 
 * This file contains several typedef used throughout the firmware.
 * There is also a forward declaration (in the form of abstract class)
 * of the \p erumby_t (which inherits from the \p erumby_base_t that is 
 * defined in this file). This is a requirement of the compiler.
 */

#include <stdint.h>

/** \brief Current machine mode. The mode is set by the \p radio_t class
 *
 * The mode as for now is selected by the remote through the lateral switch on
 * the current remote.
 * with the default configuration, we have three possible mode:
 *  - \p Auto: lower position on the remote. In this mode the low level controller
 *    accepts inputs from the i2c (Raspberry Pi)
 *  - \p Secure: central position on the remote. In this mode the motors does not
 *    move but it is still possible to read the information on i2c or
 *    interact with the Serial.
 *  - \p Manual: the higher position of the switch on the remote. In this configuration
 *    the machine accepts controls of the remote with a bypass. This function
 *    is disabled as for now, since the remote traction and steer controls does not work.
 *
 * The modes comes from the PWM of the receiver. The pin for reading the mode is
 * defined in the \p MODE. The three modes PWM value are described in the three
 * defines: \p DUTY_MODE_AUTO, \p DUTY_MODE_SECURE, and \p DUTY_MODE_MANUAL. In
 * order to have a more stable read there is an additional define, \p DUTY_MODE_OFFSET,
 * which defines a domain of PWM for each mode.
 * 
 * \warning The remote traction and steer control do not work, thus in manual mode
 * probably the car does nothing.
 * 
 * \see radio_t
 */
typedef enum erumby_mode_t {
  Auto,   /**< lower position on the remote. In this mode the low level controller
               accepts inputs from the i2c (Raspberry Pi) */
  Secure, /**< central position on the remote. In this mode the motors does not
               move but it is still possible to read the information on i2c or
               interact with the Serial */
  Manual  /**< the higher position of the switch on the remote. In this configuration
               the machine accepts controls of the remote with a bypass. This function
               is disabled as for now, since the remote traction and steer controls does not work. */
} erumby_mode_t;

typedef uint8_t pin_t;     /**< Pin type declaration (also used for map and ) */
typedef uint16_t cmd_t;    /**< PWM command type declaration */
typedef cmd_t pulse_t;     /**< PWM pulse type declaration */
typedef cmd_t output_t;    /**< Output to i2c declaration */
typedef int16_t input_t;   /**< Input from i2c declaration */
typedef uint8_t counter_t; /**< PWM Counter type declaration */
typedef uint32_t timing_t; /**< tic/toc sync timers */
typedef int16_t omega_t;   /**< types for angular speed in integer */

/**
 * \brief Forward declaration for \p erumby_t
 * 
 * This abstract class is a forward declaration for the \p erumby_t class.
 * The forward declaration allows us to have a channel for communication
 * between each component of the car and the car itself (for example for
 * raising an alarm). This approach is required by the compiler (simple
 * forward declarations cannot access methods).
 * 
 * A component that has a pointer to an istance of \p erumby_base_t
 * can only call methods that are hereby listed. The method hereby
 * listed must be purely virtual and overriden in the singleton class 
 * erumby_t.
 * 
 * \see erumby_t
 */
class erumby_base_t {
 public:
  erumby_base_t(){};
  virtual erumby_mode_t mode() = 0;
  virtual float omega_r() = 0;
  virtual float omega_l() = 0;
  virtual float omega() = 0;
  virtual const cmd_t traction() const = 0;
  virtual void traction(cmd_t v) = 0;
  virtual void speed(float v) = 0;
  virtual const cmd_t steer() const = 0;
  virtual void steer(cmd_t v) = 0;
  virtual void stop() = 0;
  
  virtual void alarm(const char* who) = 0;
  virtual void alarm(const char * who, const char * what) = 0;
};

#endif /* TYPES_HPP */