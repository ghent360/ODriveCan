#include "TaskManager.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

TEST_SUITE("TaskManager")
{
    TEST_CASE("empty state")
    {
        TaskManager tm;
        CHECK(tm.findById(1) == nullptr);
        CHECK(tm.find([](TaskNode*){ return true; }) == nullptr);
    }

    TEST_CASE("allocator OOM")
    {
        TaskManager tm;
        TaskNode* node;
        for(int i = 0; i < 32; i++) {
            node = tm.newTask(0, 0, nullptr);
            CHECK(node != nullptr);
        }
        CHECK(tm.newTask(0, 0, nullptr) == nullptr);
        tm.freeTask(node);
        CHECK(tm.newTask(0, 0, nullptr) != nullptr);
        CHECK(tm.newTask(0, 0, nullptr) == nullptr);
    }

    TEST_CASE("list OPs")
    {
        TaskManager tm;
        TaskNode* node1 = tm.newTask(1, 1000, nullptr);
        TaskNode* node2 = tm.newTask(2, 1000, nullptr);
        TaskNode* node3 = tm.newTask(3, 800, nullptr);
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
        TaskNode* firstNode = tm.find([](TaskNode*){ return true; });
        CHECK(firstNode != nullptr);
        CHECK(firstNode->id() == 2);
        tm.remove(firstNode);
        firstNode = tm.find([](TaskNode*){ return true; });
        CHECK(firstNode != nullptr);
        CHECK(firstNode->id() == 1);
        tm.remove(firstNode);
        firstNode = tm.find([](TaskNode*){ return true; });
        CHECK(firstNode != nullptr);
        CHECK(firstNode->id() == 3);
        tm.remove(firstNode);
    }

    TEST_CASE("findNext test")
    {
        TaskManager tm;
        TaskNode* node1 = tm.newTask(1, 1000, nullptr);
        TaskNode* node2 = tm.newTask(2, 1000, nullptr);
        TaskNode* node3 = tm.newTask(3, 800, nullptr);
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
}