#ifndef RADIO_T_HPP
#define RADIO_T_HPP

/**
 * \file radio_t.hpp
 * \author Davide Piscini, Matteo Ragni
 * 
 * The file implements the singleton class that handles the radio remote. 
 * The current remote sends to the receiver onboard 3 signals:
 *  - the PWM signal of the lateral switch (that is used to determine the mode)
 *  - the PWM signal that corresponds to the traction control (trigger on the remote)
 *  - the PWM signal that corresponds to the steer control (the wheel on the remote)
 * The class attaches two \p pwm_reader_t pin and a \p pwm_reader_attachable_t pin in order
 * to read the above signals.
 * 
 * The mode has three states:
 *  - lateral switch high: the machine enters in the manual mode and responds to 
 *    a manual drive using remote trigger and remote wheel (not supported!)
 *  - lateral switch middle: the machine enters in a safe mode. No control action is 
 *    sent to the actuators, the machine stops
 *  - lateral switch low: the machine runs in automatic mode and responds to the signal 
 *    arriving from the i2c connection.
 * 
 * The input of the remote trigger and remote wheel is remapped using two
 * 1-D linear interpolating lookup table. The table for the traction (converts
 * PWM input from the remote to PWM for the ESC) is defined through the two array
 * in defines \p REMOTE_MOTOR_LUT_X, \p REMOTE_MOTOR_LUT_Y (breakpoint size 
 * \p REMOTE_MOTOR_LUT_SIZE). The table for the steer (converts
 * PWM input from the remote to PWM for the ESC) is defined through the two array
 * in defines \p REMOTE_STEER_LUT_X, \p REMOTE_STEER_LUT_Y (breakpoint size 
 * \p REMOTE_STEER_LUT_SIZE). This map can be changed!
 * 
 * \warning The remote trigger and wheels does not work, thus the remote input 
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 * 
 * \warning Due to the non functioning remote the Manual mode has never been 
 * actually tested. **Test it in a safe environment before using it**.
 */

#include <Arduino.h>
#include "configurations.hpp"
#include "pwm_reader_t.hpp"
#include "lookup_table_t.hpp"
#include "types.hpp"

/** \brief Radio remote software interface
 * 
 * The file implements the singleton class that handles the radio remote. 
 * The current remote sends to the receiver onboard 3 signals:
 *  - the PWM signal of the lateral switch (that is used to determine the mode)
 *  - the PWM signal that corresponds to the traction control (trigger on the remote)
 *  - the PWM signal that corresponds to the steer control (the wheel on the remote)
 * The class attaches two \p pwm_reader_t pin and a \p pwm_reader_attachable_t pin in order
 * to read the above signals.
 * 
 * The mode has three states:
 *  - lateral switch high: the machine enters in the manual mode and responds to 
 *    a manual drive using remote trigger and remote wheel (not supported!)
 *  - lateral switch middle: the machine enters in a safe mode. No control action is 
 *    sent to the actuators, the machine stops
 *  - lateral switch low: the machine runs in automatic mode and responds to the signal 
 *    arriving from the i2c connection.
 * 
 * The input of the remote trigger and remote wheel is remapped using two
 * 1-D linear interpolating lookup table. The table for the traction (converts
 * PWM input from the remote to PWM for the ESC) is defined through the two array
 * in defines \p REMOTE_MOTOR_LUT_X, \p REMOTE_MOTOR_LUT_Y (breakpoint size 
 * \p REMOTE_MOTOR_LUT_SIZE). The table for the steer (converts
 * PWM input from the remote to PWM for the ESC) is defined through the two array
 * in defines \p REMOTE_STEER_LUT_X, \p REMOTE_STEER_LUT_Y (breakpoint size 
 * \p REMOTE_STEER_LUT_SIZE). This map can be changed!
 * 
 * The class is implemented as a singleton, since there can be only **one
 * remote** (a singleton class is a class that can have only one istance. In order
 * to do so, the constructor is made private on purpose, and to create an instance
 * a specific creator function is made available).
 * 
 * \warning The remote trigger and wheels does not work, thus the remote input 
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 * 
 * \warning Due to the non functioning remote the Manual mode has never been 
 * actually tested. **Test it in a safe environment before using it**.
 */
class radio_t {
  static radio_t * self; /**< The only instance of the radio_t class */
  pwm_reader_t motor; /**< PWM reader for the trigger input */
  pwm_reader_t steer; /**< PWM reader for the steer input */
  pwm_reader_attachable_t mode; /**< PWM reader for the mode */
  erumby_mode_t curr_mode; /**< Current mode for the machine */
  erumby_base_t* m; /**< pointer to the erumby main instance */

#ifndef REMOTE_NOT_WORKING
  lookup_table_t< cmd_t, REMOTE_MOTOR_LUT_SIZE > motor_lookup; /**< Mapping for traction */
  lookup_table_t< cmd_t, REMOTE_STEER_LUT_SIZE > steer_lookup; /**< Mapping for steer */
#endif
  
  /** \brief Provate constructor */
  radio_t(erumby_base_t* m_); 
 public:
  /** \brief Singleton constructor for radio instance
   * 
   * The class is implemented as a singleton, since there can be only **one
   * remote** (a singleton class is a class that can have only one istance. In order
   * to do so, the constructor is made private on purpose, and to create an instance
   * a specific creator function is made available).
   * 
   * The singleton constructor calls the \p radio_t constructor. If already 
   * contructed, the function returns \p self.
   * 
   * The radio starts in mode secure.
   * 
   * \param m_ pointer to erumby main instance
   * \return pointer to the singleton instance of \p radio_t
   */
  static radio_t* create_radio(erumby_base_t * m_);

  /** \brief main loop for the remote
   * 
   * This main loop reads the PWM and sets eventually the values in erumby
   * as required. The main loop is also responsible for reading the mode
   * from the remote.
   * If in manual mode it also sets the input from the remote trigger
   * and wheel
   * 
   * \warning The remote trigger and wheels does not work, thus the remote input 
   * has been disabled commenting the define \p REMOTE_NOT_WORKING
   * 
   * \warning Due to the non functioning remote the Manual mode has never been 
   * actually tested. **Test it in a safe environment before using it**.
   */
  void loop();

  /** \brief Returns the current mode
   * 
   * The mode has three states:
   *  - lateral switch high: the machine enters in the manual mode and responds to 
   *    a manual drive using remote trigger and remote wheel (not supported!)
   *  - lateral switch middle: the machine enters in a safe mode. No control action is 
   *    sent to the actuators, the machine stops
   *  - lateral switch low: the machine runs in automatic mode and responds to the signal 
   *    arriving from the i2c connection.
   * 
   * The machine starts in mode **secure**.
   * 
   * \return the current mode in the form of an enum
   */
  erumby_mode_t get_mode() { return curr_mode; }
};

#endif /* RADIO_T_HPP */