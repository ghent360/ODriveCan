/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#include "../PeriodicTimer.hpp"

#include "doctest.h"

static int numFired;
static void FireCounter(uint32_t) {
  numFired++;
}

TEST_SUITE("PeriodicTimer")
{
    TEST_CASE("test firing")
    {
        numFired = 0;
        PeriodicTimer timers[] = {
            PeriodicTimer(10, FireCounter)
        };
        StartAllTimers(timers, 1000);
        CheckAllTimers(timers, 1000);
        CHECK(numFired == 0);
        CheckAllTimers(timers, 1009);
        CHECK(numFired == 0);
        CheckAllTimers(timers, 1010);
        CHECK(numFired == 1);
        CheckAllTimers(timers, 1010);
        CHECK(numFired == 1);
        CheckAllTimers(timers, 1011);
        CHECK(numFired == 1);
        CheckAllTimers(timers, 1019);
        CHECK(numFired == 1);
        CheckAllTimers(timers, 1020);
        CHECK(numFired == 2);
    }
}