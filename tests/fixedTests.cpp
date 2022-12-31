/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include "../Fixed.hpp"

#include "doctest.h"

using F8_4 = Fixed<uint8_t, 8, 4> ;
using F8_4_b50 = Fixed<uint8_t, 8, 4, 50> ;
using S8_4 = Fixed<int8_t, 8, 4> ;
using S8_4_b50 = Fixed<int8_t, 8, 4, 50> ;
using F5_2 = Fixed<uint8_t, 5, 2> ;

TEST_SUITE("Fixed template tests")
{
    TEST_CASE("convert basic FP")
    {
        F8_4 fx1(1.25f);
        CHECK(float(fx1) == 1.25f);
        fx1 = 1.125f;
        CHECK(float(fx1) == 1.125f);
        fx1 = 1.1f;
        CHECK(float(fx1) == 1.125f);
        fx1 = 1.05f;
        CHECK(float(fx1) == 1.0625f);
    }

    TEST_CASE("convert biased FP")
    {
        F8_4_b50 fx1(51.25f);
        CHECK(float(fx1) == 51.25f);
        fx1 = 51.125f;
        CHECK(float(fx1) == 51.125f);
        fx1 = 51.1f;
        CHECK(float(fx1) == 51.125f);
        fx1 = 51.05f;
        CHECK(float(fx1) == 51.0625f);
    }

    TEST_CASE("min/max tests")
    {
        CHECK(F8_4::min() == 0.0f);
        CHECK(F8_4::max() == (16.0f - 0.0625f));
        CHECK(F8_4_b50::min() == 50.0f);
        CHECK(F8_4_b50::max() == (66.0f - 0.0625f));
        CHECK(S8_4::min() == -(8.0f - 0.0625f));
        CHECK(S8_4::max() == (8.0f - 0.0625f));
        CHECK(S8_4_b50::min() == 50.0f + -(8.0f - 0.0625f));
        CHECK(S8_4_b50::max() == (58.0f - 0.0625f));
        CHECK(F5_2::min() == 0.0f);
        CHECK(F5_2::max() == (8.0f - 0.25f));
    }

    TEST_CASE("conversion saturation tests")
    {
        F8_4 f1(-1);
        CHECK((float(f1) == 0));
        f1 = 50.0f;
        CHECK((float(f1) == (16.0f - 0.0625f)));
        F8_4_b50 f2(21.57f);
        CHECK((float(f2) == 50));
        f2 = 100;
        CHECK((float(f2) == F8_4_b50::max()));
    }
}