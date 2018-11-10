#ifndef LOOKUP_TABLE_T_HPP
#define LOOKUP_TABLE_T_HPP

/**
 * \file lookup_table.hpp
 * \author Semi efficient implementation for a Lookup Table
 *
 * The file implements a lookup table, with a linear interpolation
 * and saturation. The saturated value may be changed during construction
 * of the lookup table.
 * The points within two breakpoints are interpolated linearly.
 * The interpolation is preevaluated, and the number of the breakpoints is
 * determined during construction (this makes the lookup table predetermined
 * at construction time).
 * 
 * Evaluation requires the time for searching the mnearest breakpoint,
 * a sum and a multiplication. The longer the table the longer the
 * searching time (there is no searching cache).
 */

// Definition for algorithm testing on a computer
#ifndef __AVR__
#include <cstddef>
typedef std::size_t size_t;
#else
#include <Arduino.h>
#endif

/** \brief 1-D linear interpolating lookup table
 * 
 * The file implements a lookup table, with a linear interpolation
 * and saturation. The saturated value may be changed during construction
 * of the lookup table.
 * The points within two breakpoints are interpolated linearly.
 * The interpolation is preevaluated, and the number of the breakpoints is
 * determined during construction (this makes the lookup table predetermined
 * at construction time).
 * 
 * Evaluation requires the time for searching the mnearest breakpoint,
 * a sum and a multiplication. The longer the table the longer the
 * searching time (there is no searching cache).
 * 
 * The validity of the lookup table can be checked through the \p is_valid
 * method. In order to have a valid lookup table, the x breakpoints must
 * be **strictly monotonically increasing**.
 * 
 * It is also possible to set the saturation values. Normally, outside
 * the breakpoints domain the table returns a constant value.
 * 
 * Usage example:
 * @code
 * float x[5] = { 1, 2, 3, 4, 5 };
 * float y[5] = { 5, 4, 3, 2, 1 };
 * lookup_table_t<float, 5> f;
 * float z = f(2.5) // z is 3.5
 * @endcode
 * 
 * \warning for a fastest response the lookup object stores \p 3 * (B + 1)
 * elements of templated type \p T. Thus the space occupied by the table
 * may explode quickly.
 * 
 * \warning template type \p T must be numerical
 * 
 * \tparam T type used in the lookup table (input and output must be equal)
 * \tparam B number of breakpoints for the lookup table.
 */
template < typename T, size_t B >
class lookup_table_t {
  T x[B + 1]; /**< Stores breakpoint values for searching, increased by one */
  T m[B + 1]; /**< Stores interpolation coefficient */
  T q[B + 1]; /**< Stores offset coeffient */

  /** \brief Initialize the lookup table 
   * 
   * This function evaluates the interpolation and the offset
   * coefficients starting from the real table. With this init
   * the saturations are set to \p y[0] and \p y[B - 1]. Some 
   * constructor may override this saturation values.
   * 
   * \param x_ input points of the lookup table
   * \param y_ output points of the lookup table
   */ 
  void init(const T x_[B], const T y_[B]);

 public:
  /** \brief Empty constructor */
  lookup_table_t() { 
    for (size_t i = 0; i < B + 1; i++) {
      q[i] = 0;
      m[i] = 0;
      x[i] = 0;
    }
  }
  /** \brief Initialize the lookup table 
   * 
   * This function evaluates the interpolation and the offset
   * coefficients starting from the real table. With this constructor
   * the saturations are set to \p y[0] and \p y[B - 1].
   * 
   * \param x_ input points of the lookup table
   * \param y_ output points of the lookup table
   */ 
  lookup_table_t(const T x_[B], const T y_[B]) { 
    init(x_, y_);
  }

  /** \brief Initialize the lookup table 
   * 
   * This function evaluates the interpolation and the offset
   * coefficients starting from the real table. With this constructor
   * the saturations are both set to \p sat_. 
   * 
   * \param x_ input points of the lookup table
   * \param y_ output points of the lookup table
   * \param sat_ saturation value for evaluation outside domain
   */ 
  lookup_table_t(const T x_[B], const T y_[B], T sat_) { 
    init(x_, y_); 
    q[0] = sat_;
    q[B] = sat_;
  }

  /** \brief Initialize the lookup table 
   * 
   * This function evaluates the interpolation and the offset
   * coefficients starting from the real table. With this constructor
   * the saturations are both set to \p sat_. 
   * 
   * \param x_ input points of the lookup table
   * \param y_ output points of the lookup table
   * \param low_sat_ saturation value for evaluation below the minimum
   *        breakpoint
   * \param high_sat_ saturation value for evaluation above the maximum
   *        breakpoint
   */ 
  lookup_table_t(const T x_[B], const T y_[B], T low_sat_, T high_sat_) { 
    init(x_, y_);
    q[0] = low_sat_;
    q[B] = high_sat_;
  }

  /** \brief Copy constructor 
   * 
   * Copy constructor
   * 
   * \param other copied lookup table 
   */
  lookup_table_t(const lookup_table_t& other) { copy(other); }

  /** \brief Check if the lookup table is valid
   * 
   * To be valid, the lookup table must have **strictly monotonically
   * increasing** breakpoints.
   * 
   * \return a bool, \p true if the table is valid
   */
  const bool is_valid () const;

  /** \brief Evaluates using the table
   * 
   * The function evaluates using the table. The execution time
   * of this function depends on the legth of the lookup table. 
   * Do not use table with too many points.
   * 
   * \param z input in the lookup table
   * \return evaluated point from the lookup table
   */
  const T eval(T z) const;

  /** \brief Evaluates using the table
   * 
   * The function evaluates using the table. The execution time
   * of this function depends on the legth of the lookup table. 
   * Do not use table with too many points.
   * 
   * \param z input in the lookup table
   * \return evaluated point from the lookup table
   */
  const T operator()(T z) const { return eval(z); }

  /** \brief Minimum breakpoint value */
  const T x_min() const { return x[0]; }
  /** \brief Maximum breakpoint value */
  const T x_max() const { return x[B]; }
};

#endif /* LOOKUP_TABLE_T_HPP */