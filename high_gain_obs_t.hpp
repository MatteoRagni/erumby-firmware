#ifndef HIGH_GAIN_OBS_HPP
#define HIGH_GAIN_OBS_HPP

#include "types.hpp"

template < timing_t MILLIS >
class high_gain_obs_t {
  const static float ts = float(MILLIS) / 1000.0;
  float x[3];
  float xp[3];
  float Al[9];
  float Bl[3];

 public:
  high_gain_obs_t() : x({0}), xp({0}), Al({0}), Bl({0}) {};
  high_gain_obs_t(const float l1, const float l2, const float l3, const float epsilon) : x({0}), xp({0}) {
    gain(l1, l2, l3, epsilon);
  };

  const float operator()(const float theta) {
    xp[0] = Al[0] * x[0] + Al[1] * x[1] + Al[2] * x[2] - Bl[0] * theta;
    xp[1] = Al[3] * x[0] + Al[4] * x[1] + Al[5] * x[2] - Bl[1] * theta;
    xp[2] = Al[6] * x[0] + Al[7] * x[1] + Al[8] * x[2] - Bl[2] * theta;
    
    x[0] = xp[0];
    x[1] = xp[1];
    x[2] = xp[2];

    return x[1];
  }

  void reset() { 
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
    xp[0] = 0;
    xp[1] = 0;
    xp[2] = 0;
   }

  const float & operator[](const size_t i) const { return x[i]; }
  float & operator[](const size_t i) { return x[i]; }

 private:
  void gain(const float l1_, const float l2_, const float l3_, const float epsilon_) {
    float l1 = l1_ / epsilon_;
    float l2 = l2_ / (epsilon_ * epsilon_);
    float l3 = l3_ / (epsilon_ * epsilon_ * epsilon_);

    float ts_2 = ts * ts;
    float ts_3 = ts_2 * ts;
    float det = -l3 * ts_3 - l2 * ts_2 - l1 * ts + 1;

    Al[0] = (1) / det;
    Al[1] = (ts) / det;
    Al[2] = (ts_2) / det;
    Al[3] = (l3 * ts_2 + l2 * ts) / det;
    Al[4] = (1 - l1 * ts) / det;
    Al[5] = (-ts * (l1 * ts - 1)) / det;
    Al[6] = (l3 * ts) / det;
    Al[7] = (l3 * ts_2) / det;
    Al[8] = (-l2 * ts_2 - l1 * ts + 1) / det;

    Bl[0] = (ts * (l3 * ts_2 + l2 * ts + l1)) / det;
    Bl[1] = (ts * (l2 + l3 * ts)) / det;
    Bl[2] = (l3 * ts) / det;
  }
};

#endif /* HIGH_GIN_OBS_HPP */