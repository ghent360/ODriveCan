/*
 * Copyright (c) 2023 ghent360. See LICENSE file for details.
 */

#pragma once

#include <stdint.h>
#include <cmath>

template<uint8_t Npoints, typename fpType=double>
class BezierCurve {
public:
  BezierCurve() {
    static_assert(Npoints > 2, "Minimum 3 points are supported");
    initBinomial();
  }

  // Point index: [0..Npoints-1]
  void setPoint(uint8_t idx, fpType x, fpType y) {
    if (idx < Npoints) {
        point_x_[idx] = x;
        point_y_[idx] = y;
    }
  }

  void calculate(fpType t, fpType& x, fpType& y) const {
#if 0
    fpType oneMinusTpowers[Npoints];
    fpType tmp = 1;
    for(uint32_t idx = 0; idx < Npoints; idx++) {
        oneMinusTpowers[idx] = tmp;
        tmp *= (1 - t);
    }
    fpType Tpower = 1; // t^0 = 1
    x = 0;
    y = 0;
    for(uint32_t idx = 0; idx < Npoints; idx++) {
        fpType b = binomial_[idx] * oneMinusTpowers[Npoints - 1 - idx] * Tpower;
        x += b * point_x_[idx];
        y += b * point_y_[idx];
        Tpower *= t;
    }
#else
    fpType oneMinusTpowerN = std::pow(1 - t, Npoints - 1);
    const fpType oneOverOneMinusT = 1 / (1 - t);
    fpType Tpower = 1; // t^0 = 1
    x = 0;
    y = 0;
    for(uint32_t idx = 0; idx < Npoints; idx++) {
        fpType b = binomial_[idx] * oneOverOneMinusT * Tpower;
        x += b * point_x_[idx];
        y += b * point_y_[idx];
        Tpower *= t;
         // Avoid fp division by multiplying by 1 / (1-t)
        oneMinusTpowerN *= oneOverOneMinusT;
    }
#endif
  }
private:
  void initBinomial() {
    static constexpr uint32_t factorial[] = {
        1, 1, 2, 6, 24, 120, 720, 5040, 40320, 362880,
        3628800, 39916800, 479001600
    };

    static_assert(Npoints <= 13, "Maximum 13 points are supported");
    for (uint8_t idx = 0; idx < Npoints; idx++) {
        binomial_[idx] = 
          (fpType)factorial[Npoints - 1] /
          (factorial[idx] * factorial[Npoints - 1 - idx]);
    }
  }
  fpType binomial_[Npoints];
  fpType point_x_[Npoints];
  fpType point_y_[Npoints];
};
