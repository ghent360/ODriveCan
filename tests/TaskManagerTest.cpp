/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
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

uint32_t millis() {
    return 0;
}

static void RemoveSelfAt2000(TaskNode* self, uint32_t time) {
    numFired++;
    if (time >= 2000) {
        tm.remove(self, true);
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

    TEST_CASE("findNext test")
    {
        tm.reset();
        TaskNode* node1 = tm.newTask(1, 0, 1000, false, nullptr);
        TaskNode* node2 = tm.newTask(2, 0, 1000, false, nullptr);
        TaskNode* node3 = tm.newTask(3, 0, 800, false, nullptr);
        tm.addFront(node1);
        tm.addFront(node2);
        tm.addBack(node3);
        TaskNode* execNode = tm.findNext(1000);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 2);
        execNode = tm.findNext(999);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 3);
        execNode = tm.findNext(799);
        CHECK(execNode == nullptr);
        tm.remove(node1);
        execNode = tm.findNext(1000);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 2);
        execNode = tm.findNext(999);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 3);
        execNode = tm.findNext(799);
        CHECK(execNode == nullptr);
        tm.remove(node2);
        execNode = tm.findNext(1000);
        CHECK(execNode != nullptr);
        CHECK(execNode->id() == 3);
        execNode = tm.findNext(799);
        CHECK(execNode == nullptr);
    }

    TEST_CASE("findNext Time Overflow")
    {
        tm.reset();
        TaskNode* node1 = tm.newTask(1, 0xfffffffe, 5, false, nullptr);
        tm.addFront(node1);
        CHECK(tm.findNext(0xfffffffe) == nullptr);
        CHECK(tm.findNext(0xffffffff) == nullptr);
        CHECK(tm.findNext(0) == nullptr);
        CHECK(tm.findNext(1) == nullptr);
        CHECK(tm.findNext(2) == nullptr);
        CHECK(tm.findNext(3) != nullptr);
   }

   TEST_CASE("Simple task runner")
   {
        tm.reset();
        numFired = 0;
        TaskNode* t1 = tm.newTask(1, 1000, 10, false, FireCountTest);
        tm.addFront(t1);
        CHECK(tm.runNext(1000) == 0); // Nothing to schedule at time 1000
        CHECK(tm.runNext(1009) == 0); // Nothing to schedule at time 1009
        CHECK(tm.runNext(1010) == 1); // time since last idle = 1009-1010
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
        CHECK(tm.runNext(1000) == 0); // Nothing to schedule at time 1000
        CHECK(tm.runNext(1009) == 0); // Nothing to schedule at time 1009
        CHECK(tm.runNext(1010) == 1); // time since last idle = 1010-1009
        CHECK(numFired == 1);
        CHECK(tm.runNext(1020) == 11); // time since last idle = 1020-1009
        CHECK(numFired == 2);
        CHECK(tm.runNext(1029) == 0); // Nothing to schedule at time 1029
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
        CHECK(tm.runNext(2100) == 0);
        CHECK(numFired == 4);
   }
}