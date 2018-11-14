#ifndef COMMUNICATIONS_T_HPP
#define COMMUNICATIONS_T_HPP

/**
 * \file esc_t.hpp
 * \author Davide Piscini, Matteo Ragni
 *
 * The class implements the communication bus between the Raspberry PI
 * and the low level control. The communication as for now is in a developing stage,
 * and due to limitations in the Simulink schemes, we can only send
 * a coherent array of data (data all of the same type). As for now the data
 * in input is a \p int16_t, while the output data are \p uint16_t.
 *
 * \todo Convert the input and output structs in unions that uses float for data
 * in such a way it is not necessary to swap lsb and msb, and reduce the resource
 * for receiving data.
 *
 * The current structure for the **input data**:
 *
 * | \p indata_t field | Description                                                      |
 * |-------------------|------------------------------------------------------------------|
 * | `steering`        | PWM value to write to steering                                   |
 * | `traction`        | Value for wheel speed if positive, value for ESC pwm if negative |
 *
 * The `traction` value, if positive sends the required wheel speed for the closed loop
 * controller, while the negative value sends directly a value to the ESC.
 *
 * \warning The reference in wheel speed is:
 * \f[\omega_{ref} = \frac{\mathrm{traction}}{100}\;(rad/s)\f]
 *
 * For the PWM values, please check the file \p configurations.hpp.
 *
 * The current structure for the **output data**:
 *
 * | \p outdata_t field | Description                    |
 * |--------------------|--------------------------------|
 * | `omega_rr`         | Speed of the right wheel       |
 * | `omega_rl`         | Speed of the left wheel        |
 * | `input_esc`        | Current PWM written on the ESC |
 *
 * \warning The speed sent out is in the form:
 * \f{align}
 *  \textrm{omega\_rr} & = \mathrm{round}\left( 100 \omega_{right} \right)
 *  \textrm{omega\_rl} & = \mathrm{round}\left( 100 \omega_{left} \right)
 * \f}
 */

#include <Arduino.h>
#include <Wire.h>
#include "configurations.hpp"
#include "types.hpp"

/** \brief Class for the i2c communications in the vehicle
 *
 * The class implements the communication bus between the Raspberry PI
 * and the low level control. The communication as for now is in a developing stage,
 * and due to limitations in the Simulink schemes, we can only send
 * a coherent array of data (data all of the same type). As for now the data
 * in input is a \p int16_t, while the output data are \p uint16_t.
 *
 * \todo Convert the input and output structs in unions that uses float for data
 * in such a way it is not necessary to swap lsb and msb, and reduce the resource
 * for receiving data.
 *
 * The current structure for the **input data**:
 *
 * | \p indata_t field | Description                                                      |
 * |-------------------|------------------------------------------------------------------|
 * | `steering`        | PWM value to write to steering                                   |
 * | `traction`        | Value for wheel speed if positive, value for ESC pwm if negative |
 *
 * The `traction` value, if positive sends the required wheel speed for the closed loop
 * controller, while the negative value sends directly a value to the ESC.
 *
 * \warning The reference in wheel speed is:
 * \f[\omega_{ref} = \frac{\mathrm{traction}}{100}\;(rad/s)\f]
 *
 * For the PWM values, please check the file \p configurations.hpp.
 *
 * The current structure for the **output data**:
 *
 * | \p outdata_t field | Description                    |
 * |--------------------|--------------------------------|
 * | `omega_rr`         | Speed of the right wheel       |
 * | `omega_rl`         | Speed of the left wheel        |
 * | `input_esc`        | Current PWM written on the ESC |
 *
 * \warning The speed sent out is in the form:
 * \f{align}
 *  \textrm{omega\_rr} & = \mathrm{round}\left( 100 \omega_{right} \right)
 *  \textrm{omega\_rl} & = \mathrm{round}\left( 100 \omega_{left} \right)
 * \f}
 *
 * Since we are sending integers and the bit ordering (endianess) in the Atmel microncontroller
 * is opposite with respect to the one of the Raspberry PI, we have to swap LSB and MSB.
 * This is done in the receiving callback for the \p Wire library.
 *
 * \warning The class is implemented as a **Singleton**, since there can be only one
 * user of the i2c communication bus.
 */
class communication_t {
  /** \brief Output data structure */
  typedef struct outdata_t {
    output_t omega_rr;  /**< Rear right wheel angular speed: \f$\mathrm{round}\left( 100 \omega_{right} \right)\f$ */
    output_t omega_rl;  /**< Rear right wheel angular speed: \f$\mathrm{round}\left( 100 \omega_{left} \right)\f$ */
    output_t input_esc; /**< Current PWM value on the ESC */
  } outdata_t;

  /** \brief Input data structure */
  typedef struct indata_t {
    input_t steering; /**< Steering PWM value */
    input_t traction; /**< Wheel speed reference set point if positive, ESC PWM value if negative */
  } indata_t;

  static communication_t* self;   /**< The single instance for i2c communication */
  indata_t indata;                /**< Instance of the input structure */
  outdata_t outdata;              /**< Instance of the output structure */
  byte input[sizeof(indata_t)];   /**< Input stream */
  byte output[sizeof(outdata_t)]; /**< Output stream */
  erumby_base_t* m;               /**< Pointer to the erumby main class instance */

  /** \brief Constructor for the \p communication_t class. It is private
   *
   * The constructor for the \p communication_t class is made private in order to build
   * a singleton instance of the class, since there can be only one user for the
   * i2c communication bus.
   *
   * The constructor registers also the callbaks for sending and receiving the data.
   *
   * \param m_ erumby main instance
   */
  communication_t(erumby_base_t* m_);

 public:
  /** \brief Singleton constructor for the communication
   *
   * The static function creates a single instance and save it in the \p self
   * static variable. If the \p self variable is not \p nullptr, the value of
   * \p self is returned.
   *
   * \param m_ erumby main instance
   * \return the pointer to the singleton instance of \p communication_t
   */
  static communication_t* create_comms(erumby_base_t* m_);
  /**
   * \brief Get the singleton \p communication_t pointer
   * \return Get the singleton \p communication_t pointer
   */
  static const communication_t* get_comms();

  /** \brief Loop to run in \p erumby_t::loop_secure and \p erumby_t::loop_manual */
  void loop_secure();

  /** \brief Loop to run in \p erumby_t::loop_auto */
  void loop_auto();

  /**
   * \brief Callback to run for receiving data from Wire library
   * \param size size of the message (in bytes)
   */
  void receive(int size);

  /** \brief Callback to run when data are requested from the Wire library */
  void send();

  /** 
   * \brief Gets the last received traction value
   * \return the last received traction value
   */
  cmd_t traction() { return indata.traction; }

  /** 
   * \brief Gets the last received steering value
   * \return the last receiving steering value
   */
  cmd_t steer() { return indata.steering; }
};

#endif /* COMMUNICATIONS_T_HPP */