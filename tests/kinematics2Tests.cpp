/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include "../kinematics2.h"
#include <math.h>

#include "doctest.h"

#if 0
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

    TEST_CASE("Full forward kinematics") {
        float x, y, z;
        forwardKinematics2(0, 0, 0, x, y, z);
        forwardKinematics2(-M_PI/2, 0, 0, x, y, z);
        forwardKinematics2(-M_PI/2, 0, M_PI/2, x, y, z);
        forwardKinematics2(0, M_PI/2, 0, x, y, z);
        forwardKinematics2(0, M_PI/2, M_PI/2, x, y, z);
        forwardKinematics2(M_PI/4, -M_PI/2, 0, x, y, z);
        forwardKinematics2(M_PI/3, -M_PI/2, 0, x, y, z);
        forwardKinematics2(M_PI/2.05, -M_PI/2, 0, x, y, z);
        forwardKinematics2(M_PI/2, -M_PI/2, 0, x, y, z);

        // Lef fully extended down, hip horizontal 
        // (have to rotate the tie to point down)
        forwardKinematics2(0, -M_PI/2, 0, x, y, z);
        CHECK(fabs(x) < 0.0005);
        CHECK(fabs(y + (199.36f + 205.0f)) < 0.0005);
        CHECK(fabs(z - 107.36f) < 0.0005);
        // Leg fully extended, hip vertical up
        forwardKinematics2(-M_PI/2, -M_PI/2, 0, x, y, z);
        CHECK(fabs(x) < 0.0005);
        CHECK(fabs(y - 107.36f) < 0.0005);
        CHECK(fabs(z + (199.36f + 205.0f)) < 0.0005);
        // Leg fully extended, hip vertical down
        forwardKinematics2(M_PI/2, -M_PI/2, 0, x, y, z);
        CHECK(fabs(x) < 0.0005);
        CHECK(fabs(y + 107.36f) < 0.0005);
        CHECK(fabs(z - (199.36f + 205.0f)) < 0.0005);
        forwardKinematics2(0, -M_PI/2, M_PI/2, x, y, z);
        CHECK(fabs(x - 205.0f) < 0.0005);
        CHECK(fabs(y + 199.36f) < 0.0005);
        CHECK(fabs(z - 107.36f) < 0.0005);
        forwardKinematics2(0, -M_PI/2, -M_PI/2, x, y, z);
        CHECK(fabs(x + 205.0f) < 0.0005);
        CHECK(fabs(y + 199.36f) < 0.0005);
        CHECK(fabs(z - 107.36f) < 0.0005);
    }

    TEST_CASE("Full inverse kinematics") {
        float h, t, k;
        inverseKinematics2(0, -(199.36f + 205.0f), 107.36f, true, h, t, k);
        CHECK(fabs(h) < 0.0005);
        CHECK(fabs(t) < 0.0005); // should be -M_PI/2 to match FK
        CHECK(fabs(k) < 0.0005);
        // Can't solve Y > 0
        //inverseKinematics2(0, 107.36f, -(199.36f + 205.0f), true, th1, th2, th3);
        // Can't solve correctly edge condition if (z > 0) needs TLC.
        //inverseKinematics2(0, -107.36f, (199.36f + 205.0f), true, h, t, k);
        inverseKinematics2(205, -199.36, 107.36, true, h, t, k);
        CHECK(fabs(h) < 0.0005);
        CHECK(fabs(t) < 0.0005); // should be -M_PI/2
        CHECK(fabs(k - M_PI / 2) < 0.0005);
        inverseKinematics2(-205, -199.36, 107.36, false, h, t, k);
        CHECK(fabs(h) < 0.0005);
        CHECK(fabs(t) < 0.0005); // should be -M_PI/2
        CHECK(fabs(k + M_PI / 2) < 0.0005);
    }
}
#endif
