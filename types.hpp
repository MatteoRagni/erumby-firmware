#ifndef TYPES_HPP
#define TYPES_HPP

#include <stdint.h>

typedef uint8_t pin_t;          /**< Pin type declaration (also used for map and ) */
typedef uint16_t cmd_t;         /**< PWM command type declaration */
typedef cmd_t pulse_t;          /**< PWM pulse type declaration */
typedef cmd_t output_t;      /**< Output to i2c declaration */
typedef int16_t input_t;        /**< Input from i2c declaration */
typedef uint8_t counter_t;      /**< PWM Counter type declaration */
typedef unsigned long timing_t; /**< tic/toc sync timers */
typedef int16_t omega_t;

typedef enum erumby_mode_t { Auto, Secure, Manual } erumby_mode_t;

class erumby_base_t {
 public:
  erumby_base_t(){};
  virtual erumby_mode_t mode() = 0;
  virtual void alarm(const char* who) = 0;
  virtual omega_t omega_r() = 0;
  virtual omega_t omega_l() = 0;
  virtual const cmd_t traction() const = 0;
  virtual void traction(cmd_t v) = 0;
  virtual const cmd_t steer() const = 0;
  virtual void steer(cmd_t v) = 0;
  virtual void stop() = 0;
};

#endif /* TYPES_HPP */