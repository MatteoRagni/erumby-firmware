#include "lookup_table_t.hpp"

template < typename T, size_t B >
void lookup_table_t< T, B >::init(const T x_[B], const T y_[B]) {
  T y[B + 1];

  for (size_t i = 0; i < B; i++) {
    x[i] = x_[i];
    y[i] = y_[i];
  }
  y[B] = y_[B - 1];
  x[B] = x_[B - 1];

  for (size_t i = 1; i < B; i++) {
    if (x[i] - x[i - 1] != 0)
      m[i] = (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
    else
      m[i] = 0;
  }
  m[0] = 0;
  m[B] = 0;
  for (size_t i = 1; i < B; i++)
    q[i] = y[i] - m[i] * x[i];
  q[0] = y[0];
  q[B] = y[B];
}

template < typename T, size_t B >
const bool lookup_table_t< T, B >::is_valid() const {
  for (size_t i = 1; i < B; i++) {
    if (x[i] <= x[i - 1])
      return false;
  }
  return true;
}

template < typename T, size_t B >
const T lookup_table_t< T, B >::eval(T z) const {
  size_t i = 0;
  while ((z >= x[i]) && (i < B))
    i++;
  return q[i] + m[i] * z;
}