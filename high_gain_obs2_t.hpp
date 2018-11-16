#ifndef HIGH_GAIN_OBS2_HPP
#define HIGH_GAIN_OBS2_HPP

/**
 * \file high_gain_obs_t.hpp
 * \author Matteo Ragni, Matteo Cocetti, Davide Piscini
 *
 * The class implements an high gain observer for encoders that estimates the current
 * angle of the wheel closing the loop on the measured angle on the encoder, using the
 * following structure:
 *
 * \f{align}
 *   \dot{\hat{x}} & = A \hat{x} + E(\varepsilon) L (C \hat{x} - y) \\
 *   \hat{y} & = C x \\
 *   \dot{\hat{y}} & = C' x
 * \f}
 *
 * where:
 *
 * \f{align}
 *  A & = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \\
 *  C & = \begin{bmatrix} 1 & 0 \end{bmatrix} \\
 *  C' & = \begin{bmatrix} 0 & 1 \end{bmatrix} \\
 *  L & = \begin{bmatrix} l_1 \\ l_2 \end{bmatrix} \\
 *  E(\varepsilon) & = \begin{bmatrix}
 *    \varepsilon^{-1} & 0 \\
 *    0 & \varepsilon^{-2}\\
 *  \end{bmatrix}
 * \f}
 *
 * where the parameters can be configured using Matlab for example:
 *
 * \f[
 *  L = - \mathrm{lqr}(A^\top, C^\top, I_{2 \times 2}, 1)^\top
 * \f]
 *
 * and than using \f$\varepsilon\f$ for changing the bandwidth of the filter.
 *
 * \warning Remember that the bandwidth may be limited by the integration timestep.
 */

#include <Arduino.h>
#include "types.hpp"

/** \brief Implementation of a discretized High Gain Observer
 *
 * The class implements an high gain observer for encoders that estimates the current
 * angle of the wheel closing the loop on the measured angle on the encoder, using the
 * following structure:
 *
 * \f{align}
 *   \dot{\hat{x}} & = A \hat{x} + E(\varepsilon) L (C \hat{x} - y) \\
 *   \hat{y} & = C x \\
 *   \dot{\hat{y}} & = C' x
 * \f}
 *
 * where:
 *
 * \f{align}
 *  A & = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix} \\
 *  C & = \begin{bmatrix} 1 & 0 \end{bmatrix} \\
 *  C' & = \begin{bmatrix} 0 & 1 \end{bmatrix} \\
 *  L & = \begin{bmatrix} l_1 \\ l_2\end{bmatrix} \\
 *  E(\varepsilon) & = \begin{bmatrix}
 *    \varepsilon^{-1} & 0 \\
 *    0 & \varepsilon^{-2}
 *  \end{bmatrix}
 * \f}
 *
 * where the parameters can be configured using Matlab for example:
 *
 * \f[
 *  L = - \mathrm{lqr}(A^\top, C^\top, I_{2 \times 2}, 1)^\top
 * \f]
 *
 * and than using \f$\varepsilon\f$ for changing the bandwidth of the filter.
 *
 * \warning Remember that the bandwidth may be limited by the integration timestep.
 *
 * \tparam MILLIS discretization time step in milliseconds
 */
template < timing_t MILLIS >
class high_gain_obs2_t {
  const static float ts = float(MILLIS) / 1000.0; /**< Time step of the filter */
  const static size_t state_size = 2;             /**< State size for the observer */
  float x[state_size];                            /**< Internal state of the filter */
  float xp[state_size];                           /**< next step of the filter, required for the implicit integration */
  float Al[state_size * state_size];              /**< Implicit discretization of the filter, matrix \f$A_L\f$ */
  float Bl[state_size];                           /**< Implicit discretization of the filter, matrix \f$B_L\f$ */

 public:
  /** \brief Empty constructor, it initialize an empty filter */
  high_gain_obs2_t() : x({0}), xp({0}), Al({0}), Bl({0}){};
  /** \brief Constructor with parameters
   *
   * The constructor evaluates also the discretized version of the matrices.
   * The discretization is done using a Backward Euler:
   * \f[
   *  s = \frac{z - 1}{t_s z}
   * \f]
   *
   * \param l1_ observer parameter for state 1
   * \param l2_ observer parameter for state 2
   * \param epsilon_ high gain value (usually in \f$(0, 1)\f$)
   */
  high_gain_obs2_t(const float l1_, const float l2_, const float epsilon_);

  /** \brief Evaluates the next step of the filter
   *
   * Receives a new observation to evaluate a new step using the implicit step.
   * It returns:
   * \f[
   *  C' \hat{x} = \dot{\hat{y}} 
   * \f]
   *
   * \param y last observation
   * \return the derivative of the input estimated by the high gain
   */
  const float operator()(const float y);

  /** \brief Resets the internal state of the filter */
  void reset();

  /** 
   * \brief Attribute reader for the internal state of the filter 
   * \param i index of the i-th state of the filter
   * \return the value of the i-th state of the filter
   */
  const float& operator[](const size_t i) const { return x[i]; }
  /** 
   * \brief Attribute accessor for the internal state of the filter 
   * \param i index of the i-th state of the filter
   * \return a settable reference to the i-th state of the filter
   */
  float& operator[](const size_t i) { return x[i]; }

 private:
  /** \brief Evaluates the discretization of the filter
   *
   * This function evaluates the discretization of the following dynamical system:
   * \f{align}
   *   \dot{\hat{x}} & = (A + E(\varepsilon) L C) \hat{x} - E(\varepsilon) L y \\
   *   \hat{y} & = C x
   * \f}
   * with the Backward Euler:
   * \f[
   *  s = \frac{z - 1}{t_s z}
   * \f]
   * Such that:
   * \f[
   *   \hat{x}_k = A_L \hat{x}_{k - 1} + B_L y_k
   * \f]
   * with:
   * \f{align}
   *   A_L & = (I_{2 \times 2} - t_s (A + E(\varepsilon) L C))^{-1} \\
   *   B_L & = A_L E(\varepsilon) L t_s
   * \f}
   *
   * \param l1_ observer parameter for state 1
   * \param l2_ observer parameter for state 2
   * \param epsilon_ high gain value (usually in \f$(0, 1)\f$)
   */
  void discretize(const float l1_, const float l2_, const float epsilon_);
};

#endif /* HIGH_GIN_OBS2_HPP */