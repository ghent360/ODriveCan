#include "TaskManager.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

TEST_SUITE("TaskManager")
{
    TEST_CASE("empty state")
    {
        TaskManager tm;
        CHECK(tm.findById(1) == nullptr);
        CHECK(tm.findFirst([](TaskNode*){ return true; }) == nullptr);
    }

    TEST_CASE("allocator OOM")
    {
        TaskManager tm;
        TaskNode* node;
        for(int i = 0; i < 32; i++) {
            node = tm.newTask(0, 0, 0, nullptr);
            CHECK(node != nullptr);
        }
        CHECK(tm.newTask(0, 0, 0, nullptr) == nullptr);
        tm.freeTask(node);
        CHECK(tm.newTask(0, 0, 0, nullptr) != nullptr);
        CHECK(tm.newTask(0, 0, 0, nullptr) == nullptr);
    }

    TEST_CASE("list OPs")
    {
        TaskManager tm;
        TaskNode* node1 = tm.newTask(1, 0, 1000, nullptr);
        TaskNode* node2 = tm.newTask(2, 0, 1000, nullptr);
        TaskNode* node3 = tm.newTask(3, 0, 800, nullptr);
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
        TaskManager tm;
        TaskNode* node1 = tm.newTask(1, 0, 1000, nullptr);
        TaskNode* node2 = tm.newTask(2, 0, 1000, nullptr);
        TaskNode* node3 = tm.newTask(3, 0, 800, nullptr);
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
        TaskManager tm;
        TaskNode* node1 = tm.newTask(1, 0xfffffffe, 5, nullptr);
        tm.addFront(node1);
        CHECK(tm.findNext(0xfffffffe) == nullptr);
        CHECK(tm.findNext(0xffffffff) == nullptr);
        CHECK(tm.findNext(0) == nullptr);
        CHECK(tm.findNext(1) == nullptr);
        CHECK(tm.findNext(2) == nullptr);
        CHECK(tm.findNext(3) != nullptr);
   }
}