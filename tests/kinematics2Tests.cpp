/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include "../kinematics2.h"
#include <math.h>

#include "doctest.h"


TEST_SUITE("Kinematics2") {
    TEST_CASE("forward rest position") {
        float x, y;
        forwardKinematics(M_PI / 4, - M_PI / 2, x, y);
        CHECK(fabs(x - -3.98808289f) < 1E-8);
        CHECK(fabs(y - 318.42572f) < 1E-8);
    }

    TEST_CASE("forward extremes") {
        float x, y;
        // Vertical down
        forwardKinematics(0, 0, x, y);
        CHECK(fabs(x) < 1E-8);
        CHECK(fabs(y - (float)(199.36 + 205 + 32.5)) < 1E-8);
        // Vertical + horizontal - back facing
        forwardKinematics(0, M_PI / 2, x, y);
        CHECK(fabs(x - (float)(205)) < 1E-8);
        CHECK(fabs(y - (float)(199.36 + 32.5)) < 1E-4);
        // Vertical + horizontal - front facing
        forwardKinematics(0, -M_PI / 2, x, y);
        CHECK(fabs(x + (float)(205)) < 1E-8);
        CHECK(fabs(y - (float)(199.36 + 32.5)) < 1E-4);
        // Horizontal - back facing + Vertical
        forwardKinematics(M_PI/2, -M_PI/2, x, y);
        CHECK(fabs(x - (float)(199.36)) < 1E-8);
        CHECK(fabs(y - (float)(205 + 32.5)) < 1E-4);
        // Horizontal - front facing + Vertical
        forwardKinematics(-M_PI/2, M_PI/2, x, y);
        CHECK(fabs(x + (float)(199.36)) < 1E-8);
        CHECK(fabs(y - (float)(205 + 32.5)) < 1E-4);
    }

    TEST_CASE("inverse rest position") {
        float th1, th2;
        inverseKinematics(-3.98898289, 318.42572, th1, th2);
        CHECK(fabs(th1 - (M_PI/4)) < 1E-5);
        CHECK(fabs(th2 - (-M_PI/2)) < 1E-5);
    }

    TEST_CASE("inverse extremes") {
        float th1, th2;
        // Vertical down
        inverseKinematics(0, 199.36 + 205 + 32.5, th1, th2);
        CHECK(fabs(th1) < 0.0005);
        CHECK(fabs(th2) < 0.0005);
        // Vertical + horizontal - back facing
        // Gives alternate solution
        // Vertical + horizontal - front facing
        inverseKinematics(-205, 199.36 + 32.5, th1, th2);
        CHECK(fabs(th1) < 0.0005);
        CHECK(fabs(th2 + (M_PI/2)) < 0.0005);
    }
}