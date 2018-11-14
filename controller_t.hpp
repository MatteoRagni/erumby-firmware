#ifndef ESC_CONTROLLER_HPP
#define ESC_CONTROLLER_HPP

/**
 * \file controller_t.hpp
 * \author Matteo Ragni, Davide Piscini, Matteo Cocetti
 * 
 * The file contains the code employed in the ESC control. The ESC control
 * receives a \p float with the reference for the wheel speed and try to 
 * set the PWM pulse witdh in order to obtain such value.
 * 
 * The scheme of the controller follows:
 * @code
 * 
 *                                                          +------------+
 *                                             u in (0,1)   |            |       Output to ESC PWM pin
 *                                           +------------->+ PWM_map(u) +------------------------------>
 *                 +--------------+          |              |            |
 *                 |              | u_ff     |              +------------+
 *        +------->+ PHI_INV(w_r) +------+   |
 *        |        |              |      |   |
 *        |        +--------------+      |   |                  Smith Predictor with a copy of the plant
 *        |                              |   |
 *        |        +--------------+      |   |   +--------+     +------------------+     +----------+
 *        | +   e  |              | u_fb V+  |   |        |  q  |                  |  x  |          |
 *  w_r --+->O-----+ PI Ctrl      +----->O---+-->+ SAT(u) +---->+ dx = -a x + a q  +--+->+ exp(-ds) +----+
 *           ^-    |              |     +        |        |     |                  |  |  |          |    |
 *           |     +--------------+              +--------+     +------------------+  |  +----------+    |
 *           |                                                                        |                  |
 *           |                                                           +--------+   |   Delay          |
 *           |+                                                          |        |   |                  |
 *           O<----------------------------------------------------------+ PHI(w) +<--+ x                |
 *          +^                             w_sp                          |        |                      |
 *           |                                                           +--------+                      |
 *           |                                                                                           |
 *           |                                                           +--------+                      |
 *           |                                                           |        |                      |
 * w_hg ---->O<----------------------------------------------------------+ PHI(w) +<---------------------+
 *          + -                            w_sp (delayed)                |        |     x (delayed)
 *                                                                       +--------+
 * @endcode
 * 
 * The controller is composed by several parts, and in particular we have as input to the scheme:
 *  - `w_r` that is the reference set point
 *  - `w_hg` that is the estimation coming from the high gain observers in the \p encoder_t. To
 *     be precise \f$ \omega_{hg} = \frac{1}{2} ( \omega_{hg,left} + \omega_{hg,right} )\f$
 *  - `w_sp` is the prediction of the speed of the wheel for the system without delay predicted by
 *    the Smith predictor. The plant is modeled as a 1 state wiener model:
 *    \f[
 *      \dot{x} = - a x + a \mathrm{sat}(u)
 *    \f]
 *    with the nonlinear output:
 *    \f[
 *      \omega_{sp} = \phi(u) = \frac{\sqrt{c_1^2 + 4 c_2 u}}{2 c_2} 
 *    \f]
 *    the parameters \f$ a, c_1, c_2 \f$ should be identified and inserted in the \p configure.hpp
 *    file.
 *  - `u` is the output to be sent to the esc as a PWM.
 * 
 * | Param         | Define              | Description                           |
 * |---------------|---------------------|---------------------------------------|
 * | \f$ a \f$     | `CTRL_MODEL_A`      | Dynamical system pole                 |
 * | \f$ c_1 \f$   | `CTRL_NONLIN_A`     | Non linearity first coefficient       |
 * | \f$ c_2 \f$   | `CTRL_NONLIN_B`     | Non linearity second coefficient      |
 * | \f$ d \f$     | `CTRL_SYSTEM_DELAY` | Delay: (\f$mod(d,t_s) = 0\f$! (in ms) |
 * | \f$ t_s \f$   | `LOOP_TIMING`       | Time step for integration (in ms)     |
 * | \f$ k_p \f$   | `CTRL_KP`           | PI controller proportional gain       |
 * | \f$ k_i \f$   | `CTRL_KI`           | PI controller integrative gain        | 
 *    
 * **DELAY and LOOP_TIMING**: the controller has been built with the idea of running in the real
 * time loop, which runs approximatively a 250Hz (4 ms). The Delay identified for the system is nominally
 * 80 ms. Please notice that the integer division between delay and loop timing must have no residuals
 * (`CTRL_SYSTEM_DELAY % LOOP_TIMING == 0`), in order to discretize correctly the delay.
 * 
 * \warning The delay is a characteristic of this particular system. It is not possible to eliminate it 
 * via software. 
 * 
 * \warning All the hardcoded constants are concentrated in the class \p controller_t!
 * 
 * \see controller_t
 */

#include <Arduino.h>
#include "configurations.hpp"
#include "cyclic_array_t.hpp"
#include "types.hpp"

/** \brief Class wich implements a PI controller
 * 
 * The class implements a PI controller with an Backward Euler discretization:
 * \f[
 *   s = \frac{z - 1}{t_s z}
 * \f]
 * The constructor takes as arguments the proportional and integrative gains
 * and evaluates internally the discretization. Do not discretize the gains manually.
 * 
 * The internal recursion will be:
 * \f{align}
 *   x_{k} & = x_{k - 1} + t_s e{k} \\
 *   u_{k} & = k_i x_{k - 1} + (k_p + t_s k_i) e_{k}
 * \f{align}
 * 
 * Usage example:
 * @code
 * static const timing_t ts = 4; // ms
 * pi_ctrl_t<ts> ctrl(1.0, 0.0); // Proportional only controller with kp = 1
 *                               // and sample time 4ms
 * 
 * void real_time_loop() {
 *   float reference = get_reference();    // get the reference
 *   float measure = get_measure();        // output of the system
 *   float u = ctrl(reference - measure);  // evaluate your new control
 *   set_control(u);                       // Set your control
 * }
 * @endcode
 * 
 * \tparam MILLIS the discretization time in ms. Should be equal to the real time loop timing
 */
template < timing_t MILLIS >
class pi_ctrl_t {
  const static float ts = float(MILLIS) / 1000.0; /**< Time step in seconds */
  float ei; /**< Integral of the error */
  float kp; /**< \f$k_p = k_{p,in} + t_s k_{i,in} \f$: discretized proportional gain */
  float ki; /**< \f$k_i = k{i,in}\f$: discretized integrative gain */

 public:
  /** \brief Empty constructor, sets the gains to zero */
  pi_ctrl_t() : ei(0) { gain(0, 0); }
  
  /** \brief Normal constructor, evaluates the gain and discretize
   * 
   * This constructor takes the gains as input and evaluates the gains for 
   * the discretized version of the controller.
   * 
   * \param kp_ \f$ k_{p,in} \f$: proportional gain of the controller
   * \param ki_ \f$ k_{i,in} \f$: integrative gain of the controller
   */
  pi_ctrl_t(const float kp_, const float ki_) : ei(0) { gain(kp_, ki_); }

  /** \brief Updates the gain of the controller
   * 
   * Changes the gain of the controller evaluating the following
   * discretized gain:
   * 
   * \f{align}
   * k_p & = k_{p,in} + t_s k_{i,in} \\
   * k_i & = k_{i,in}
   * \f{align}
   * 
   * \param kp_ \f$ k_{p,in} \f$: proportional gain of the controller
   * \param ki_ \f$ k_{i,in} \f$: integrative gain of the controller 
   */ 
  void gain(const float kp_, const float ki_) {
    ki = ki_;
    kp = kp_ + ts * ki_;
  }

  /** \brief Evaluate control using error
   * 
   * Evaluates the next control action using the current error.
   * The error is the difference between setpoint (\f$ r \f$) and 
   * current reading (\f$ y \f$).
   * 
   * \param e current error (\f$r - y\f$)
   * \return the control action
   */
  const float operator()(const float e) {
    float u = ki * ei + kp * e;
    ei += ts * e;
    return u;
  }

  /** \brief reset the internal state of the controller */
  void reset() { ei = 0; }
  /** \brief reset the internal state of the controller 
   * 
   * \param ei_ new value of the state
   */
  void reset(const float ei_) { ei = ei_; }
};

/** \brief Dicretization of the time delay
 * 
 * The time delay is a \p cyclic_array_t that has a memory size
 * which is \f$ n = \mathrm{floor}(d / t_s) \f$. The discretized
 * system is:
 * 
 * \f{align}
 *  x_{1}^{k+1} &= x_2^{k} \\
 *  x_{2}^{k+1} &= x_3^{k}  \\
 *  & \vdots
 *  x_{n-1}^{k+1} &= x_n^{k} \\
 *  x_n^{k+1} &= u
 * \f{align}
 * 
 * Using templates for the creation of this kind of delay has an advantage
 * in terms of code efficiency, but as drawback the delay is fixed at 
 * compile time. This also implies that the following should be respected.
 * 
 * \f[
 * \mathrm{mod}(d^{(ms)}, t_s^{(ms)}) = 0
 * \f]
 * 
 * \tparam MILLIS discretization time in millisecond
 * \tparam DELAY delay in milliseconds
 */
template < timing_t MILLIS, timing_t DELAY >
using time_delay_t = cyclic_array_t< float, DELAY / MILLIS >;

/** \brief Smith predictor
 * 
 * An implementation of a smith predictor for the following linear plant:
 * \f{align}
 *   \dot{x}(t) & = - a x(t) + a \mathrm{sat}(u(t - d)) \\
 *   y(t) & = \phi(x(t))
 * \f{align}
 * This class is taylored made for our ESC system. The integration of this 
 * system is performed with a Backward Euler
 * \f$
 *  s = \frac{z-1}{t_s z}
 * \f$
 * and the recursion follows:
 * \f{align}
 *  q_{k} & = \mathrm{sat}_{[0,1]}(t_{k}) \\
 *  x_{k} & = a_{sp} x_{k-1} + b_{sp} q_{k} \\
 *  y_{k} & = \phi(x_{k}) = \frac{\sqrt{c_1^2 + 4 c_2 x_{k}} - c_1}{2 c_2}
 * \f{align}
 * where:
 * \f{align}
 *  a_{sp} = (1 + a t_s)^{-1}
 *  b_{sp} = a_{sp} a t_s
 * \f{align}
 * 
 * The actual output in the control loop, for a discretization of the delay 
 * in a cyclic array with size \f$ n = \mathrm{floor}(d / t_s) \f$, follows:
 * \f{align}
 *  \omega &= y_{k - n} \\
 *  \omega_{predict} &= y{k} 
 * \f{align}
 * 
 * \warning The non linearity is a **virtual** method. Thus it should
 * be redefined in the controller.
 *  
 * \tparam MILLIS discretization time in millisecond
 * \tparam DELAY delay in milliseconds
 */
template < timing_t MILLIS, timing_t DELAY >
class smith_predictor_t {
  const static float ts = float(MILLIS) / 1000.0; /**< Time step in seconds */
  const static float d = float(DELAY) / 1000.0; /**< Delay in seconds */ 
  float a_sp; /**< state gain for discretization */
  float b_sp; /**< input gain for discretization */
  time_delay_t< MILLIS, DELAY > delay; /**< Delay system */
  
 public:
  /** \brief Empty constructor, gain to zero */
  smith_predictor_t() : a_sp(0), b_sp(0), delay(0){};
  /** \brief Constructor, which sets the constants for the dynamical system.
   * 
   * The constructor evaluates:
   * \f{align}
   *  a_{sp} = (1 + a t_s)^{-1}
   *  b_{sp} = a_{sp} a t_s
   * \f{align}
   * which are the discretized (with Backward Euler) counterpart for the linear
   * plant:
   * \f[
   *  \dot{x} = -a x + a u
   * \f]
   * 
   * \param a the \f$ a \f$ of the dynamical system
   */
  smith_predictor_t(float a) : delay(0) { gain(a); }

  /** \brief Main loop for the Smith predictor
   * 
   * The main loop of the Smith predictor executes the following recursion:
   * \f[
   *  x_{k} = a_{sp} x_{k-1} + b_{sp} \mathrm{sat}_{[0,1]}(u)
   * \f]
   * 
   * \param u the last input to the dynamical system
   */
  const void operator()(const float u) {
    float q = u;
    if (q < 0.0)
      q = 0.0;
    if (q > 1.0)
      q = 1.0;
    delay.push_back(a_sp * delay.back() + b_sp * q);
  }

  /** \brief Output non linearity
   * 
   * The output of the smith predictor is subject to a non-linearity
   * 
   * \warning In order to have a better generalization of the smith predictor
   * the non linearity is a virtual function, with a trivial implementation (identity).
   * The user should write her/his own implementation in the actual controller
   * class, where the non linearity will be specific for the application.
   * 
   * \param u input of the non linearity
   * \return the output of the non linearity \f$ \phi(u) \f$
   */
  virtual const float phi(const float u) const { return u; };

  /** \brief The value of the output in the internal model (with delay)
   * 
   * \return the value of the output in the internal model
   */
  const float state() const { return phi(delay.front()); }
  /** \brief The value of the output prediction in the internal model (without delay)
   * 
   * \return the value of the output prediction in the internal model
   */
  const float state_predict() const { return phi(delay.back()); }
  /** \brief resets the internal model delay and dynamical system to 0 */
  const void reset() { delay.fill(0); }

 private:
 /** \brief Sets the constants for the dynamical system.
   * 
   * The function sets:
   * \f{align}
   *  a_{sp} = (1 + a t_s)^{-1}
   *  b_{sp} = a_{sp} a t_s
   * \f{align}
   * which are the discretized (with Backward Euler) counterpart for the linear
   * plant:
   * \f[
   *  \dot{x} = -a x + a u
   * \f]
   * 
   * \param a the \f$ a \f$ of the dynamical system
   */
  void gain(const float a) {
    a_sp = 1 / (1 + a * ts);
    b_sp = a_sp * a * ts;
  }

};

/** \brief The actual ESC controller
 * 
 *  The class implements the ESC control. The ESC control
 * receives a \p float with the reference for the wheel speed and try to 
 * set the PWM pulse witdh in order to obtain such value.
 * 
 * The scheme of the controller follows:
 * @code
 * 
 *                                                          +------------+
 *                                             u in (0,1)   |            |       Output to ESC PWM pin
 *                                           +------------->+ PWM_map(u) +------------------------------>
 *                 +--------------+          |              |            |
 *                 |              | u_ff     |              +------------+
 *        +------->+ PHI_INV(w_r) +------+   |
 *        |        |              |      |   |
 *        |        +--------------+      |   |                  Smith Predictor with a copy of the plant
 *        |                              |   |
 *        |        +--------------+      |   |   +--------+     +------------------+     +----------+
 *        | +   e  |              | u_fb V+  |   |        |  q  |                  |  x  |          |
 *  w_r --+->O-----+ PI Ctrl      +----->O---+-->+ SAT(u) +---->+ dx = -a x + a q  +--+->+ exp(-ds) +----+
 *           ^-    |              |     +        |        |     |                  |  |  |          |    |
 *           |     +--------------+              +--------+     +------------------+  |  +----------+    |
 *           |                                                                        |                  |
 *           |                                                           +--------+   |   Delay          |
 *           |+                                                          |        |   |                  |
 *           O<----------------------------------------------------------+ PHI(w) +<--+ x                |
 *          +^                             w_sp                          |        |                      |
 *           |                                                           +--------+                      |
 *           |                                                                                           |
 *           |                                                           +--------+                      |
 *           |                                                           |        |                      |
 * w_hg ---->O<----------------------------------------------------------+ PHI(w) +<---------------------+
 *          + -                            w_sp (delayed)                |        |     x (delayed)
 *                                                                       +--------+
 * @endcode
 * 
 * The controller is composed by several parts, and in particular we have as input to the scheme:
 *  - `w_r` that is the reference set point
 *  - `w_hg` that is the estimation coming from the high gain observers in the \p encoder_t. To
 *     be precise \f$ \omega_{hg} = \frac{1}{2} ( \omega_{hg,left} + \omega_{hg,right} )\f$
 *  - `w_sp` is the prediction of the speed of the wheel for the system without delay predicted by
 *    the Smith predictor. The plant is modeled as a 1 state wiener model:
 *    \f[
 *      \dot{x} = - a x + a \mathrm{sat}(u)
 *    \f]
 *    with the nonlinear output:
 *    \f[
 *      \omega_{sp} = \phi(u) = \frac{\sqrt{c_1^2 + 4 c_2 u}}{2 c_2} 
 *    \f]
 *    the parameters \f$ a, c_1, c_2 \f$ should be identified and inserted in the \p configure.hpp
 *    file.
 *  - `u` is the output to be sent to the esc as a PWM.
 * 
 * | Param         | Define              | Description                           |
 * |---------------|---------------------|---------------------------------------|
 * | \f$ a \f$     | `CTRL_MODEL_A`      | Dynamical system pole                 |
 * | \f$ c_1 \f$   | `CTRL_NONLIN_A`     | Non linearity first coefficient       |
 * | \f$ c_2 \f$   | `CTRL_NONLIN_B`     | Non linearity second coefficient      |
 * | \f$ d \f$     | `CTRL_SYSTEM_DELAY` | Delay: (\f$mod(d,t_s) = 0\f$! (in ms) |
 * | \f$ t_s \f$   | `LOOP_TIMING`       | Time step for integration (in ms)     |
 * | \f$ k_p \f$   | `CTRL_KP`           | PI controller proportional gain       |
 * | \f$ k_i \f$   | `CTRL_KI`           | PI controller integrative gain        | 
 *    
 * **DELAY and LOOP_TIMING**: the controller has been built with the idea of running in the real
 * time loop, which runs approximatively a 250Hz (4 ms). The Delay identified for the system is nominally
 * 80 ms. Please notice that the integer division between delay and loop timing must have no residuals
 * (`CTRL_SYSTEM_DELAY % LOOP_TIMING == 0`), in order to discretize correctly the delay.
 * 
 * \warning The delay is a characteristic of this particular system. It is not possible to eliminate it 
 * via software. 
 * 
 * \warning This class is **taylored made for our applications and contains several hardcoded constants**.
 * It also implements as static methods the non linearities (direct and inverse) wich are used in the 
 * feed forward controller and in the Smith predictor.
 */ 
class controller_t {
  
 public:
  /** \brief Implementation of the non linearity
   * 
   * \f[
   *   \omega = \phi(u) = \frac{\sqrt{c_1^2 + 4 c_2 u}}{2 c_2}
   * \f]
   * 
   * \warning this is a static function and it is shared with the Smith predictor 
   * used by the controller
   * 
   * \param u input for the non linearity
   * \return output of the non linearity
   */
  static const float phi(const float u) {
    return (sqrt((CTRL_NONLIN_A * CTRL_NONLIN_A) + (4 * CTRL_NONLIN_B) * u) - CTRL_NONLIN_A)/(2 * CTRL_NONLIN_B);
  }

  /** \brief Implementation of the inverse of the non linearity
   * 
   * \f[
   *   u = \phi^{-1}(\omega) = c_1 \omega^2 + c_2 \omega
   * \f]
   * 
   * \warning this is a static function.
   * 
   * \param omega input for the inverse of the non linearity
   * \return output of the inverse of the non linearity
   */
  static const float phi_inv(const float omega) {
    return CTRL_NONLIN_A * omega + CTRL_NONLIN_B * omega * omega;
  }

 private:
  /** \brief A specific Smith predictor with the ESC non linearity
   * 
   * This smith predictor is specific for our ESC, with the identified non linearity
   * hardcoded inside (actually it is the \f$ \phi(u) \f$ implemented in \p controller_t
   * as a static function).
   */
  class esc_sp_t : public smith_predictor_t<LOOP_TIMING, CTRL_SYSTEM_DELAY> {
    /**
     * \brief System non linearity
     * \f[
     *   \omega = \phi(u) = \frac{\sqrt{c_1^2 + 4 c_2 u}}{2 c_2}
     * \f]
     * \param u the imput for the non linearity
     * \return the output for the non linearity
     */ 
    const float phi(const float u) const override { return controller_t::phi(u); }
   public:
    /** \brief Empty constructor */
    esc_sp_t() : smith_predictor_t<LOOP_TIMING, CTRL_SYSTEM_DELAY>() {}
    /** 
     * \brief Constructor with pole 
     * \param a the pole of the model
     */
    esc_sp_t(const float a) : smith_predictor_t<LOOP_TIMING, CTRL_SYSTEM_DELAY>() {}
  };

  pi_ctrl_t< LOOP_TIMING > pi; /**< PI controller block */
  esc_sp_t sp; /**< Smith predictor block */

 public:
  /** \brief Empty constructor
   * 
   * \warning It uses the hardcoded constants of the configuration file.
   */
  controller_t() : pi(pi_ctrl_t<LOOP_TIMING>(CTRL_KP, CTRL_KI)), sp(esc_sp_t(CTRL_MODEL_A)) {}

  /** \brief Main loop of the controller
   * 
   * Evaluates the reference tracking error through the smith predictor (in the 
   * block scheme the signal `e`), evaluates the feed forward and closed loop
   * control action and updates the smith predictor.
   * 
   * \param reference required reference
   * \param measure measure from the system (or an observer)
   * \return the current value of the input for controlling the speed
   */
  const float operator()(const float reference, const float measure) {
    float e_omega = reference - (measure - sp.state() + sp.state_predict());
    float u = controller_t::phi_inv(reference) + pi(e_omega);  // u_ff + u_fb
    sp(u);
    return u;
  }

  /** \brief Resets the internal state of the controller */
  const void reset() {
    sp.reset();
    pi.reset();
  }
};

#endif /* ESC_CONTROLLER_HPP */