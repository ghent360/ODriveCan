/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
 */
#include "../ValueWithChangeDetection.hpp"

#include "doctest.h"

TEST_SUITE("ValueWithChangeDetection template tests")
{
    TEST_CASE("basic int test")
    {
        ValueWithChangeDetection<int> v1;
        CHECK(v1 == 0);
        CHECK(v1.changed() == false);
        v1 = 0;
        CHECK(v1 == 0);
        CHECK(v1.changed() == false);
        v1 = 1;
        CHECK(v1 == 1);
        CHECK(v1.changed() == true);
        v1.changeReset();
        CHECK(v1 == 1);
        CHECK(v1.changed() == false);
    }
    TEST_CASE("basic float test")
    {
        ValueWithChangeDetection<float> v1;
        CHECK(v1 == 0.0f);
        CHECK(v1.changed() == false);
        v1 = 0.0f;
        CHECK(v1 == 0);
        CHECK(v1.changed() == false);
        v1 = 1;
        CHECK(v1 == 1.0f);
        CHECK(v1.changed() == true);
        v1.changeReset();
        CHECK(v1 == 1.0f);
        CHECK(v1.changed() == false);
        v1 = 1.000001f;
        CHECK(v1 == 1.0f);
        CHECK(v1.changed() == false);
    }
}
