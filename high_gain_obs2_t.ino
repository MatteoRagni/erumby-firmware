#include "high_gain_obs2_t.hpp"

template < timing_t MILLIS >
high_gain_obs2_t< MILLIS >::high_gain_obs2_t(const float l1_, const float l2_, const float epsilon_)
    : x({0}), xp({0}) {
  discretize(l1_, l2_, epsilon_);
};

template < timing_t MILLIS >
const float high_gain_obs2_t< MILLIS >::operator()(const float y) {
  xp[0] = Al[0] * x[0] + Al[1] * x[1] - Bl[0] * y;
  xp[1] = Al[2] * x[0] + Al[3] * x[1] - Bl[1] * y;

  x[0] = xp[0];
  x[1] = xp[1];

  return x[1];
}

template < timing_t MILLIS >
void high_gain_obs2_t< MILLIS >::reset() {
  x[0] = 0;
  x[1] = 0;
  xp[0] = 0;
  xp[1] = 0;
}

template < timing_t MILLIS >
void high_gain_obs2_t< MILLIS >::discretize(const float l1_, const float l2_, const float epsilon_) {
  float l1 = l1_ / epsilon_;
  float l2 = l2_ / (epsilon_ * epsilon_);

  float ts_2 = ts * ts;
  float det = l2 * ts_2 + l1 * ts - 1;

  Al[0] = (-1) / det;
  Al[1] = (-ts) / det;
  Al[2] = (- l2 * ts) / det;
  Al[3] = (l1 * ts - 1) / det;

  Bl[0] = (-ts * l1 - ts_2 * l2) / det;
  Bl[1] = (-ts * l2) / det;
}