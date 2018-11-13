#ifndef CONFIGURATIONS_HPP
#define CONFIGURATIONS_HPP

#define ISR_HOW ISR_NOBLOCK  // ISR_BLOCK  Abilita la ISR delle porte
#define I2C_ADDR 0x03        // define slave address

// Pins Arduino - Encoders
#define L_WHEEL_ENCODER 53  // 1 bit del registro  0x01
#define R_WHEEL_ENCODER 52  // 2 bit del registro  0x02

// Pins Arduino - Receiver
#define STEERING A8  // 1 bit del registro 0x01
#define TRACTION A9  // 2 bit del registro 0x02
#define MODE_PIN 2

// Pins Arduino - Motors
#define ESC 11            // Traction motor Arduino -> ESC
#define SERVO 12          // Stearing motor Arduino -> ESC
#define PWM_FREQUENCY 71  // Frequency for PWM in Hz

// STEARING - DUTY CYCLE VALUES
#define DUTY_STEARING_DX 1052
#define DUTY_STEARING_IDLE 1476
#define DUTY_STEARING_SX 1890

// Previous values in 16 bit
// Nominal Value = 6881
// Boundaries = +/- 27% nominal value
#define DUTY_SERVO_DX 5024
#define DUTY_SERVO_MIDDLE 6881
#define DUTY_SERVO_SX 8738

// MOTOR - DUTY CYCLE VALUES
#define DUTY_MOTOR_MAX 2032
#define DUTY_MOTOR_IDLE 1500
#define DUTY_MOTOR_IDLE_SAFE 1340
#define DUTY_MOTOR_MIN 1000

// Previous values in 16 bit
// Nominal Value = 7012
// Boundaries = +/- 20% nominal value
#define DUTY_ESC_MAX 8412
#define DUTY_ESC_MAX_SAFE 7200
#define DUTY_ESC_IDLE 7010
#define DUTY_ESC_MIN 5608

// MODE - DUTY CYCLE VALUES
#define DUTY_MODE_MANUAL 2024
#define DUTY_MODE_SECURE 1504
#define DUTY_MODE_AUTO 980
#define DUTY_MODE_OFFSET 75

#define LOOP_TIMING 4
#define SERIAL_SPEED 250000

#define ULONG_PI 3141592

#define ERROR_LED_PORT 13

#define REMOTE_NOT_WORKING
#ifndef REMOTE_NOT_WORKING
#define REMOTE_MOTOR_LUT_SIZE 3
#define REMOTE_MOTOR_LUT_X \
  { DUTY_MOTOR_MIN, DUTY_MOTOR_IDLE_SAFE, DUTY_MOTOR_MAX }
#define REMOTE_MOTOR_LUT_Y \
  { DUTY_ESC_IDLE, DUTY_ESC_IDLE, DUTY_ESC_MAX_SAFE }
#define REMOTE_STEER_LUT_SIZE 3
#define REMOTE_STEER_LUT_X \
  { DUTY_STEARING_DX, DUTY_STEARING_IDLE, DUTY_STEARING_SX }
#define REMOTE_STEER_LUT_Y \
  { DUTY_SERVO_DX, DUTY_SERVO_MIDDLE, DUTY_SERVO_SX }
#endif

// Controller configurations
#define CTRL_SYSTEM_DELAY 80 // CTRL_SYSTEM_DELAY % LOOP_TIMING must be == 0!
#define CTRL_KP  0.005
#define CTRL_KI  0.01
#define CTRL_MODEL_A   3.17

#define CTRL_NONLIN_A  0.00118
#define CTRL_NONLIN_B  1.532e-05


#define HG_L1 -2.424214
#define HG_L2 HG_L1
#define HG_L3 -1.0
#define HG_EPSILON 0.1
#define M_PI 3.1415926536

#endif /* CONFIGURATIONS_HPP */