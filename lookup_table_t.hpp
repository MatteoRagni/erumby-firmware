#ifndef LOOKUP_TABLE_T_HPP
#define LOOKUP_TABLE_T_HPP

/**
 * \file lookup_table.hpp
 * \author Semi efficient implementation for a Lookup Table
 *
 * The file implements a lookup table, with a linear interpolation
 * and saturation. The saturated value may be changed during construction
 * of the lookup table.
 */

#ifndef __AVR__
#include <cstddef>
typedef std::size_t size_t;
#endif

template < typename T, size_t B >
class lookup_table_t {
  T x[B + 1];
  T m[B + 1];
  T q[B + 1];

 public:
  void init(const T x_[B], const T y_[B]) {
    T y[B + 1];

    for (size_t i = 0; i < B; i++) {
      x[i] = x_[i];
      y[i] = y_[i];
    }
    y[B] = y_[B - 1];
    x[B] = x_[B - 1];

    for (size_t i = 1; i < B; i++)
      m[i] = (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
    m[0] = 0;
    m[B] = 0;
    for (size_t i = 1; i < B; i++)
      q[i] = y[i] - m[i] * x[i];
    q[0] = y[0];
    q[B] = y[B];
  }

  void init(const T x_[B], const T y_[B], const T sat_) {
    init(x_, y_);
    q[0] = sat_;
    q[B] = sat_;
  }

  void init(const T x_[B], const T y_[B], const T low_sat_, const T high_sat_) {
    init(x_, y_);
    q[0] = low_sat_;
    q[B] = high_sat_;
  }

  lookup_table_t(const T x_[B], const T y_[B]) { init(x_, y_); }

  lookup_table_t(const T x_[B], const T y_[B], T sat_) { init(x_, y_, sat_); }

  lookup_table_t(const T x_[B], const T y_[B], T low_sat_, T high_sat_) { init(x_, y_, low_sat_, high_sat_); }

  lookup_table_t() {
    for (size_t i = 0; i < B + 1; i++) {
      x[i] = 0;
      m[i] = 0;
      q[i] = 0;
    }
  }

  inline bool is_valid() {
    for (size_t i = 1; i < B; i++) {
      if (x[i] < x[i - 1])
        return false;
    }
    return true;
  }

  T eval(T z) {
    size_t i = 0;
    while ((z >= x[i]) && (i < B))
      i++;
    return q[i] + m[i] * z;
  }

  inline T x_min() { return x[0]; }
  inline T x_max() { return x[B]; }
};

#endif /* LOOKUP_TABLE_T_HPP */