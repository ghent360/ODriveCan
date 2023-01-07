/*
 * Copyright (c) 2023 ghent360. See LICENSE file for details.
 */

#pragma once

#include <stdint.h>

static constexpr uint32_t factorial[] = {
    1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880,
    3628800, 39916800, 479001600
};

template<uint8_t Npoints, typename fp=double>
class BezierCurve {
public:
  BezierCurve() {
    static_assert(Npoints > 2, "Minimum 3 points are supported");
    static_assert(Npoints <= 13, "Maximum 13 points are supported");
    initBinomial();
  }

  // Point index: [0..Npoints-1]
  void setPoint(uint8_t idx, fp x, fp y) {
    if (idx < Npoints) {
        point_x_[idx] = x;
        point_y_[idx] = y;
    }
  }

  void calculate(fp t, fp& x, fp& y) {
    fp oneMinusTpowers[Npoints];
    fp tmp = 1;
    for(uint32_t idx = 0; idx < Npoints; idx++) {
        oneMinusTpowers[idx] = tmp;
        tmp *= (1 - t);
    }
    fp Tpower = 1; // t^0 == 1
    x = 0;
    y = 0;
    for(uint32_t idx = 0; idx < Npoints; idx++) {
        fp b = binomial_[idx] * oneMinusTpowers[Npoints - 1 - idx] * Tpower;
        x += b * point_x_[idx];
        y += b * point_y_[idx];
        Tpower *= t;
    }
  }
private:
  void initBinomial() {
    for (uint8_t idx = 0; idx < Npoints; idx++) {
        binomial_[idx] = 
          (fp)factorial[Npoints - 1] /
          (factorial[idx] * factorial[Npoints - 1 - idx]);
    }
  }
  fp binomial_[Npoints];
  fp point_x_[Npoints];
  fp point_y_[Npoints];
};
