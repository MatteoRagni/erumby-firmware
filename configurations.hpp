#ifndef CONFIGURATIONS_HPP
#define CONFIGURATIONS_HPP

/**
 * \file configurations.hpp
 * \author Davide Piscini, Matteo Ragni
 *
 * this file contains the documentetions of the define.
 */

/**
 * \def I2C_ADDR
 *
 * define the address of the board for the i2c communications.
 */
#define I2C_ADDR 0x03

/**
 * \def L_WHEEL_ENCODER
 *
 * Defines the pin on the board where the left encoder is attached.
 * This pin is used in the \p pwm_reader_t for reading the port B and
 * according to arduinomega doc is 0x01 pin (PB0).
 * This pin is use in the class \p pwm_reader_t.
 */
#define L_WHEEL_ENCODER 53

/**
 * \def R_WHEEL_ENCODER
 *
 * Defines the pin on the board where the right encoder is attached.
 * This pin is used in the \p pwm_reader_t for reading the port B and
 * according to arduinomega doc is 0x02 pin (PB1).
 * This pin is use in the class \p pwm_reader_t.
 */
#define R_WHEEL_ENCODER 52

/**
 * \def ENCODER_QUANTIZATION
 * 
 * The define must match the quantization of the encoder (number of transparent
 * windows). The more accurate this value, the less the error of the high gain
 * observer for the encoder.
 */
#define ENCODER_QUANTIZATION 100

/**
 * \def STEERING
 *
 * Defines the pin on the board where the steering signal from reciver is attached.
 * This signal is sent by the receiver which in turn reads it from the radio control
 * This pin is used in the \p pwm_reader_t for reading the port K and
 * according to arduinomega doc is 0x01 pin (PK0).
 * This pin is use in the class \p pwm_reader_t.
 *
 * \warning The remote trigger and wheels does not work, thus the remote input
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 *
 * \warning Due to the non functioning remote the Manual mode has never been
 * actually tested. **Test it in a safe environment before using it**.
 */
#define STEERING A8

/**
 * \def TRACTION
 *
 * Defines the pin on the board where the traction signal from reciver is attached.
 * This signal is sent by the receiver which in turn reads it from the radio control
 * This pin is used in the \p pwm_reader_t for reading the port K and
 * according to arduinomega doc is 0x02 pin (PK1).
 * This pin is use  in the class \p pwm_reader_t.
 *
 * \warning The remote trigger and wheels does not work, thus the remote input
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 *
 * \warning Due to the non functioning remote the Manual mode has never been
 * actually tested. **Test it in a safe environment before using it**.
 */
#define TRACTION A9

/**
 * \def MODE_PIN
 *
 * Defines the pin on the board where the mode signal from reciver is attached.
 * This signal is sent by the receiver which in turn reads it from the radio control.
 * This pin is used in the \p pwm_reader_t for reading the digital pin 2, which is a pin
 * with a attach interrupt function avaible, according to arduinomega doc.
 * This pin is use in the class \p pwm_reader_attachable_t for read the lateral swich from the
 * radio command.
 *
 */
#define MODE_PIN 2

/**
 * \def ESC
 *
 * Defines the pin on the board where the esc signal send to the
 * receiver is attached. This signal is sent to the receiver which in turn sends
 * it to the esc for control the motor.
 * This pin is use in the class \p esc_t, for sends the pwm signal to esc
 */
#define ESC 11

/**
 * \def SERVO
 *
 * Defines the pin on the board where the servo signal send to the
 * receiver is attached. This signal is sent to the receiver which in turn sends
 * it to the servo for control the motor.
 * This pin is use in the class \p servo_t, for sends the pwm signal to servo motor
 */
#define SERVO 12

/**
 * \def PWM_FREQUENCY
 *
 * Defines the pwm frequency for sending the signal to the receiver.
 * This frequency is set according to the receiver manual.
 */
#define PWM_FREQUENCY 71

/**
 * \def DUTY_SERVO_DX
 *
 * Defines the maximum pwm allowed value for turn right the servo.
 * This value is - 27% of the nominal value (DUTY_SERVO_MIDDLE).
 */
#define DUTY_SERVO_DX 5024

/**
 * \def DUTY_SERVO_MIDDLE
 *
 * Defines the pwm value for set the idle state of the servo.
 */
#define DUTY_SERVO_MIDDLE 6881

/**
 * \def DUTY_SERVO_SX
 *
 * Defines the maximum pwm allowed value for turn left the servo.
 * This value is + 27% of the nominal value (DUTY_SERVO_MIDDLE).
 */
#define DUTY_SERVO_SX 8738

/**
 * \def DUTY_ESC_MAX
 *
 * Defines the maximum pwm allowed value to be send to esc for normal mode
 * of the motor.
 * This value is +20% of the nominal value (DUTY_ESC_IDLE).
 */
#define DUTY_ESC_MAX 8412

/**
 * \def DUTY_ESC_IDLE
 *
 * Defines the pwm value for set the idle state of the motor.
 */
#define DUTY_ESC_IDLE 7010

/**
 * \def DUTY_ESC_MIN
 *
 * Defines the minimum pwm allowed value to be send to esc for reverse mode
 * of the motor.
 * This value is -20% of the nominal value (DUTY_ESC_IDLE).
 */
#define DUTY_ESC_MIN 5608

/**
 * \def DUTY_MODE_MANUAL
 *
 * Defines the pwm value read from the recever for set the mode of vehicle control.
 * This pwm value is read if the lateral switch is set to high position:
 * the machine enters in the manual mode and responds to
 * a manual drive using remote trigger and remote wheel (not supported!)
 *
 * \warning The remote trigger and wheels does not work, thus the remote input
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 *
 * \warning Due to the non functioning remote the Manual mode has never been
 * actually tested. **Test it in a safe environment before using it**.
 */
#define DUTY_MODE_MANUAL 2024

/**
 * \def DUTY_MODE_SECURE
 *
 * Defines the pwm value read from the recever for set the secure mode
 * of vehicle control. This pwm value is read if the
 * lateral switch is set to middle position: the machine enters in a safe mode.
 * No control action is sent to the actuators, the machine stops.
 */
#define DUTY_MODE_SECURE 1504

/**
 * \def DUTY_MODE_AUTO
 *
 * Defines the pwm value read from the recever for set the automatic mode
 * of vehicle control. This pwm value is read if the
 * lateral switch is set to low position: the machine runs in automatic mode and responds to the signal
 * arriving from the i2c connection.
 */
#define DUTY_MODE_AUTO 980

/**
 * \def DUTY_MODE_OFFSET
 *
 * Defines the pwm value of the baundary for checking what mode are selected
 * This valuse is use in \p radio_t.
 */
#define DUTY_MODE_OFFSET 75

/**
 * \def LOOP_TIMING
 *
 * Define of loop timing of the soft real time, the value is set in ms, and it
 * is use also in the discretization of the system.
 */
#define LOOP_TIMING 4

/**
 * \def SERIAL_SPEED
 *
 * Define the serial speed baud rate
 */
#define SERIAL_SPEED 115200

/**
 * \def ERROR_LED_PORT
 *
 * define the digital pin to set high of the arduino mega when a \p alarm
 * is called.
 */
#define ERROR_LED_PORT 13

/**
 * \def REMOTE_NOT_WORKING
 *
 * define the disabilitation of the manual mode for control the car.
 *  * \warning The remote trigger and wheels does not work, thus the remote input
 * has been disabled commenting the define \p REMOTE_NOT_WORKING
 *
 * \warning Due to the non functioning remote the Manual mode has never been
 * actually tested. **Test it in a safe environment before using it**.
 */
#define REMOTE_NOT_WORKING
#ifndef REMOTE_NOT_WORKING

/**
 * \def DUTY_STEERING_DX
 *
 * Defines the maximum pwm allowed value read by radio controller
 * for right steering input.
 * This value is - 27% of the nominal value (DUTY_STEERING_IDLE).
 */
#define DUTY_STEERING_DX 1052

/**
 * \def DUTY_STEERING_DX
 *
 * Defines the idle pwm value read by radio controller
 * for zero input.
 */
#define DUTY_STEERING_IDLE 1476

/**
 * \def DUTY_STEERING_SX
 *
 * Defines the maximum pwm allowed value read by radio controller
 * for left steering input.
 * This value is + 27% of the nominal value (DUTY_STEERING_IDLE).
 */
#define DUTY_STEERING_SX 1890

/**
 * \def DUTY_MOTOR_MAX
 *
 * Defines the maximum pwm allowed value read by radio controller
 * for straight motor input.
 * This value is + 20% of the nominal value (DUTY_MOTOR_IDLE).
 */
#define DUTY_MOTOR_MAX 2032

/**
 * \def DUTY_MOTOR_IDLE
 *
 * Defines the idle pwm value read by radio controller
 * for zero input.
 */
#define DUTY_MOTOR_IDLE 1340

/**
 * \def DUTY_MOTOR_MIN
 *
 * Defines the maximum pwm allowed value read by radio controller
 * for reverse motor input.
 * This value is - 20% of the nominal value (DUTY_MOTOR_IDLE).
 */
#define DUTY_MOTOR_MIN 1000

/**
 * \def DUTY_MOTOR_LUT_SIZE
 *
 * Defines the numbers of breakpoint in the lookup table (\p look_up_table_t)
 * for mapping the MOTOR value, who is reading by radio comand, in ESC value who
 * is send to the recever.
 */
#define REMOTE_MOTOR_LUT_SIZE 3

/**
 * \def DUTY_MOTOR_LUT_X
 *
 * Define the x value of the lookup table (\p look_up_table_t)
 * for mapping the MOTOR value, who is read by radio comand, in ESC value who
 * is send to the recever.
 */
#define REMOTE_MOTOR_LUT_X \
  { DUTY_MOTOR_MIN, DUTY_MOTOR_IDLE, DUTY_MOTOR_MAX }

/**
 * \def DUTY_MOTOR_LUT_Y
 *
 * Define the y value of the lookup table (\p look_up_table_t)
 * for mapping the MOTOR value, who is read by radio comand, in ESC value who
 * is send to the recever.
 */
#define REMOTE_MOTOR_LUT_Y \
  { DUTY_ESC_IDLE, DUTY_ESC_IDLE, DUTY_ESC_MAX_SAFE }

/**
 * \def DUTY_STEER_LUT_SIZE
 *
 * Defines the numbers of breakpoint in the lookup table (\p look_up_table_t)
 * for mapping the STEERING value, who is reading by radio comand, in SERVO value who
 * is send to the recever.
 */
#define REMOTE_STEER_LUT_SIZE 3

/**
 * \def DUTY_MOTOR_LUT_X
 *
 * Define the x value of the lookup table (\p look_up_table_t)
 * for mapping the STEERING value, who is reading by radio comand, in SERVO value who
 * is send to the recever.
 */
#define REMOTE_STEER_LUT_X \
  { DUTY_STEERING_DX, DUTY_STEERING_IDLE, DUTY_STEERING_SX }

/**
 * \def DUTY_MOTOR_LUT_Y
 *
 * Define the x value of the lookup table (\p look_up_table_t)
 * for mapping the STEERING value, who is reading by radio comand, in SERVO value who
 * is send to the recever.
 */
#define REMOTE_STEER_LUT_Y \
  { DUTY_SERVO_DX, DUTY_SERVO_MIDDLE, DUTY_SERVO_SX }
#endif

/**
 * \def CTRL_SYSTEM_DELAY
 *
 * Define the delay of the mechanical system. The actuation of ESC system
 * has a delay and this value is use in the \p controller_t for the smith observer.
 */
#define CTRL_SYSTEM_DELAY 80

/**
 * \def CTRL_KP
 *
 * Define the value of proportional gain in the PI controller (\p controller_t)
 */
#define CTRL_KP 0.01

/**
 * \def CTRL_KI
 *
 * Define the value of integral gain in the PI controller (\p controller_t)
 */
#define CTRL_KI 0.01

/**
 * \def CTRL_MODEL_A
 *
 * Define the value of the pole in the dynamical system. This value is use in the \p controller_t
 * for the smith observer.
 */
#define CTRL_MODEL_A 3.17

/**
 * \def CTRL_NONLIN_A
 *
 * Define the value of non linearity first coefficient. According to equation.
 * \f[
 *   u = \phi^{-1}(\omega) = c_1 \omega^2 + c_2 \omega
 * \f]
 * This value is use in the \p controller_t for the smith observer.
 */
#define CTRL_NONLIN_A 0.00118

/**
 * \def CTRL_NONLIN_B
 *
 * Define the value of non linearity first coefficient. According to equation.
 * \f[
 *   u = \phi^{-1}(\omega) = c_1 \omega^2 + c_2 \omega
 * \f]
 * This value is use in the \p controller_t for the smith observer.
 */
#define CTRL_NONLIN_B 1.532e-05

/**
 * \def HG_L1
 *
 * Define observer parameter for state 1 in the \p high_gain_obs_t
 */
#define HG_L1 -5

/**
 * \def HG_L2
 *
 * Define observer parameter for state 2 in the \p high_gain_obs_t
 */
#define HG_L2 -6

/**
 * \def HG_L3
 *
 * Define observer parameter for state 3 in the \p high_gain_obs_t
 * If this parameter is not defined, the code will compile the stack
 * with an high gain observer of order 2.
 */
//#define HG_L3 -1.0

/**
 * \def HG_EPSILON
 *
 * Define observer epsilon high gain value (usually in \f$ (0, 1) \f$) in the \p high_gain_obs_t
 */
#define HG_EPSILON 0.1

/**
 * \def M_PI
 *
 * Define of pi constant
 */
#define M_PI 3.1415926536

#endif /* CONFIGURATIONS_HPP */