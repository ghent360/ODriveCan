/*
 * Copyright (c) 2022-2023 ghent360@iqury.us. See LICENSE file for details.
*/
#include "../TaskManager.hpp"

#include "doctest.h"

TaskManager tm;

static uint32_t numFired;
static void FireCountTest(TaskNode*, uint32_t) {
    numFired++;
}

uint32_t micros() {
    return 0;
}

static uint32_t currentTime = 0;
uint32_t millis() {
    return currentTime;
}

static void RemoveSelfAt2000(TaskNode* self, uint32_t time) {
    numFired++;
    if (time >= 2000) {
        tm.remove(self, true);
    }
}

static uint32_t recursiveShouldExit;
static void RecursiveRun(TaskNode*, uint32_t) {
    while (currentTime < recursiveShouldExit) {
        tm.runNext();
        currentTime++;
    }
}

TEST_SUITE("TaskManager")
{
    TEST_CASE("empty state")
    {
        tm.reset();
        CHECK(tm.findById(1) == nullptr);
        CHECK(tm.findFirst([](TaskNode*){ return true; }) == nullptr);
    }

    TEST_CASE("allocator OOM")
    {
        tm.reset();
        TaskNode* node;
        for(int i = 0; i < 32; i++) {
            node = tm.newTask(0, 0, 0, false, nullptr);
            CHECK(node != nullptr);
        }
        CHECK(tm.newTask(0, 0, 0, false, nullptr) == nullptr);
        tm.freeTask(node);
        CHECK(tm.newTask(0, 0, 0, false, nullptr) != nullptr);
        CHECK(tm.newTask(0, 0, 0, false, nullptr) == nullptr);
    }

    TEST_CASE("list OPs")
    {
        tm.reset();
        TaskNode* node1 = tm.newTask(1, 0, 1000, false, nullptr);
        TaskNode* node2 = tm.newTask(2, 0, 1000, false, nullptr);
        TaskNode* node3 = tm.newTask(3, 0, 800, false, nullptr);
        tm.addFront(node1);
        CHECK(tm.findById(1) != nullptr);
        tm.remove(node1);
        CHECK(tm.findById(1) == nullptr);
        tm.addBack(node1);
        CHECK(tm.findById(1) != nullptr);
        tm.remove(node1);
        tm.addFront(node1);
        tm.addFront(node2);
        tm.addBack(node3);
        // Order should ne node2->node1->node3
        TaskNode* firstNode = tm.findFirst([](TaskNode*){ return true; });
        CHECK(firstNode != nullptr);
        CHECK(firstNode->id() == 2);
        tm.remove(firstNode);
        firstNode = tm.findFirst([](TaskNode*){ return true; });
        CHECK(firstNode != nullptr);
        CHECK(firstNode->id() == 1);
        tm.remove(firstNode);
        firstNode = tm.findFirst([](TaskNode*){ return true; });
        CHECK(firstNode != nullptr);
        CHECK(firstNode->id() == 3);
        tm.remove(firstNode);
    }

    TEST_CASE("findNextRunnable test")
    {
        tm.reset();
        TaskNode* node1 = tm.newTask(1, 0, 1000, false, nullptr);
        TaskNode* node2 = tm.newTask(2, 0, 1000, false, nullptr);
        TaskNode* node3 = tm.newTask(3, 0, 800, false, nullptr);
        tm.addFront(node1);
        tm.addFront(node2);
        tm.addBack(node3);
        TaskNode* execNode = tm.findNextRunnable(1000);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 2);
        execNode = tm.findNextRunnable(999);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 3);
        execNode = tm.findNextRunnable(799);
        CHECK(execNode == nullptr);
        tm.remove(node1);
        execNode = tm.findNextRunnable(1000);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 2);
        execNode = tm.findNextRunnable(999);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 3);
        execNode = tm.findNextRunnable(799);
        CHECK(execNode == nullptr);
        tm.remove(node2);
        execNode = tm.findNextRunnable(1000);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 3);
        execNode = tm.findNextRunnable(799);
        CHECK(execNode == nullptr);
    }

    TEST_CASE("findNextRunnable Time Overflow")
    {
        tm.reset();
        TaskNode* node1 = tm.newTask(1, 0xfffffffe, 5, false, nullptr);
        tm.addFront(node1);
        CHECK(tm.findNextRunnable(0xfffffffe) == nullptr);
        CHECK(tm.findNextRunnable(0xffffffff) == nullptr);
        CHECK(tm.findNextRunnable(0) == nullptr);
        CHECK(tm.findNextRunnable(1) == nullptr);
        CHECK(tm.findNextRunnable(2) == nullptr);
        CHECK(tm.findNextRunnable(3) != nullptr);
   }

   TEST_CASE("Simple task runner")
   {
        tm.reset();
        numFired = 0;
        TaskNode* t1 = tm.newTask(1, 1000, 10, false, FireCountTest);
        tm.addFront(t1);
        currentTime = 1000;
        tm.runNext(); // Nothing to schedule at time 1000
        CHECK(numFired == 0);
        currentTime = 1009;
        tm.runNext(); // Nothing to schedule at time 1009
        CHECK(numFired == 0);
        currentTime = 1010;
        tm.runNext(); // Run FireCountTest
        CHECK(numFired == 1);
        // Check all tasks have been removed
        CHECK(tm.findFirst([](TaskNode*){ return true; }) == nullptr);
   }

   TEST_CASE("Periodic task runner")
   {
        tm.reset();
        numFired = 0;
        TaskNode* t1 = tm.newTask(1, 1000, 10, true, FireCountTest);
        tm.addFront(t1);
        currentTime = 1000;
        tm.runNext(); // Nothing to schedule at time 1000
        CHECK(numFired == 0);
        currentTime = 1009;
        tm.runNext(); // Nothing to schedule at time 1009
        CHECK(numFired == 0);
        currentTime = 1010;
        tm.runNext(); // Run FireCountTest
        CHECK(numFired == 1);
        currentTime = 1011;
        tm.runNext(); // Nothing for time 1011
        CHECK(numFired == 1);
        currentTime = 1020;
        tm.runNext(); // Run FireCountTest again
        CHECK(numFired == 2);
        currentTime = 1029;
        tm.runNext(); // Nothing runs for 1029
        CHECK(numFired == 2);
   }

   TEST_CASE("Periodic task can delete itself")
   {
        tm.reset();
        numFired = 0;
        TaskNode* t1 = tm.newTask(1, 1000, 10, true, RemoveSelfAt2000);
        tm.addFront(t1);
        tm.runNext(1100);
        CHECK(numFired == 1);
        tm.runNext(1200);
        CHECK(numFired == 2);
        tm.runNext(1900);
        CHECK(numFired == 3);
        tm.runNext(2000);
        CHECK(numFired == 4); // task should have removed itself now
        CHECK(tm.findFirst([](TaskNode*){ return true; }) == nullptr);
        tm.runNext(2100);
        CHECK(numFired == 4);
   }

   TEST_CASE("Recursive run, except ourselves")
   {
        tm.reset();
        numFired = 0;
        TaskNode* t1 = tm.newTask(1, 1000, 2, true, FireCountTest);
        TaskNode* t2 = tm.newTask(2, 1000, 10, true, RecursiveRun);
        tm.addFront(t1);
        tm.addFront(t2);
        recursiveShouldExit = 1100;
        currentTime = 1002;
        tm.runNext();
        CHECK(numFired == 1);
        currentTime = 1004;
        tm.runNext();
        CHECK(numFired == 2);
        currentTime = 1006;
        tm.runNext();
        CHECK(numFired == 3);
        currentTime = 1008;
        tm.runNext();
        CHECK(numFired == 4);
        currentTime = 1010;
        tm.runNext();
        CHECK(currentTime == 1100);
        CHECK(numFired == 49);
        // Time is still 1100, we should have the FireCountTest ready to run
        tm.runNext();
        CHECK(numFired == 50); // Yes we did.
        // Time is still 1100, nothing is scheduled at this time anymore.
        tm.runNext();
        CHECK(numFired == 50);

        currentTime = 1110;
        // We should be running the RecursiveRun again, but it should simply exit
        // because the current time > 1100
        tm.runNext();
        CHECK(numFired == 50);
        // Time is still 1110, we should now run FireCountTest.
        tm.runNext();
        CHECK(numFired == 51); // Yes we did
        // No more tasks scheduled for 1110
        tm.runNext();
        CHECK(numFired == 51);
   }
}
