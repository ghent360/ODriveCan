/*
 * Copyright (c) 2022 ghent360. See LICENSE file for details.
*/
#pragma once
#include <stdint.h>

// Returns the current time in milliseconds
#ifndef ARDUINO
extern "C" uint32_t millis();
#endif

// This is internal class structure for each task.
class TaskNode {
public:
  // A task is implemented by a function. It is expected that
  // each task would not do blocking calls.
  //
  // Teach task gets passed the TaskNode structure and the current
  // time in milliseconds
  typedef void (*Callback)(TaskNode* self, uint32_t tineNow);

  uint32_t id() const { return id_; }
  void run(uint32_t timeNow) {
    if (cb_) {
      cb_(this, timeNow);
    }
  }

  friend class TaskManager;
private:
  bool in_use_;
  TaskNode *prev_;
  TaskNode *next_;
  uint32_t id_;
  uint32_t sched_time_;
  uint32_t interval_;
  bool is_periodic_;
  Callback cb_;
};

class TaskManager {
public:    
  TaskManager() {
    reset();
  }

  // Reset the task manager to empty state.
  void reset() {
    task_list_start_ = task_list_end_ = nullptr;
    last_idle_time_ = 0;
    for(auto& node: pool_) {
      node.in_use_ = false;
    }
  }

  // Allocate a new task. Return nullptr if we are out of task nodes.
  TaskNode* newTask(
      uint32_t id,      // ID for the task, so it can be found later
      uint32_t timeNow, // the current time in milliseconds
      uint32_t interval,// how long to wait before executing the task
      bool isPeriodic,  // periodic tasks get re-scheduled automatically
      TaskNode::Callback cb) { // The function that implements the task
    for(auto& node : pool_) {
      if (!node.in_use_) {
        node.in_use_ = true;
        node.id_ = id;
        node.sched_time_ = timeNow;
        node.interval_ = interval;
        node.cb_ = cb;
        node.is_periodic_ = isPeriodic;
        node.prev_ = nullptr;
        node.next_ = nullptr;
        return &node;
      }
    }
    HandleOOM("Task pool is full");
    return nullptr;
  }

  // Simplified interface for newTask for non periodic tasks
  TaskNode* newSimpleTask(
      uint32_t id,
      uint32_t interval,
      TaskNode::Callback cb) {
    return newTask(id, millis(), interval, false, cb);
  }

  // Simplified interface for newTask for periodic tasks
  TaskNode* newPeriodicTask(
      uint32_t id,
      uint32_t interval,
      TaskNode::Callback cb) {
    return newTask(id, millis(), interval, true, cb);
  }

  // Free a previously allocated task. Note before calling freeTask
  // the task should be removed from the scheduler.
  //
  // In practice you should never call this method directly.
  void freeTask(TaskNode* node) {
    if (node) {
      node->in_use_ = false;
      node->sched_time_ = 0;
      node->interval_ = 0;
      node->cb_ = nullptr;
      node->prev_ = nullptr;
      node->next_ = nullptr;
    }
  }

  // Add a task to the front of the scheduler queue.
  void addFront(TaskNode* node) {
    if (task_list_start_) {
      task_list_start_->prev_ = node;
    }
    node->next_ = task_list_start_;
    node->prev_ = nullptr;
    task_list_start_ = node;
    if (!task_list_end_) {
      task_list_end_ = node;
    }
  }

  // Add a task to the back of the scheduler queue.
  void addBack(TaskNode* node) {
    if (task_list_end_) {
      task_list_end_->next_ = node;
    }
    node->prev_ = task_list_end_;
    node->next_ = nullptr;
    task_list_end_ = node;
    if (!task_list_start_) {
      task_list_start_ = node;
    }
  }

  // Remove a task from the queue and optionally free it.
  void remove(TaskNode* node, bool free = false) {
    if (!node) return;
    if (node->prev_) {
      node->prev_->next_ = node->next_;
    }
    if (task_list_start_ == node) {
      task_list_start_ = node->next_;
    }
    if (node->next_) {
      node->next_->prev_ = node->prev_;
    }
    if (task_list_end_ == node) {
      task_list_end_ = node->prev_;
    }
    node->next_ = nullptr;
    node->prev_ = nullptr;
    if (free) {
      freeTask(node);
    }
  }

  template<typename Functor>
  TaskNode* findFirst(Functor cond) {
    TaskNode* node = task_list_start_;
    while (node) {
      if (cond(node)) {
        return node;
      }
      node = node->next_;
    }
    return nullptr;
  }

  template<typename Functor>
  TaskNode* findLast(Functor cond) {
    TaskNode* node = task_list_end_;
    while (node) {
      if (cond(node)) {
        return node;
      }
      node = node->prev_;
    }
    return nullptr;
  }

  // Find task with specified ID in the scheduler queue.
  TaskNode* findById(uint32_t id) {
    return findFirst([id](const TaskNode* node) {
      return node->id_ == id;
    });
  }

  // Finds the next task that can me scheduled at this time or return
  // nullptr if no such task exists.
  TaskNode* findNext(uint32_t time) {
    return findFirst([time](const TaskNode* node) {
      return ((time - node->sched_time_) >= node->interval_);
    });
  }

  // Run the first task that can be scheduled. If the task is periodic, re-schedule
  // it for the next time. If not, remove the task from the queue and free it.
  //
  // Return the time in ms since we could not schedule any tasks.
  uint32_t runNext(uint32_t time) {
    TaskNode *next = findNext(time);
    if (next) {
      next->run(time);
      if (next->in_use_) { // Check if the task has not been freed
        if (next->is_periodic_) {
          next->sched_time_ = time;
        } else {
          remove(next, true);
        }
      }
    } else {
      last_idle_time_ = time;
    }
    return time - last_idle_time_;
  }
private:
  void HandleOOM(const char*) {
  }
 
  // We allocate 32 tasks total. If you need more change this number.
  TaskNode pool_[32];
  TaskNode *task_list_start_;
  TaskNode *task_list_end_;
  uint32_t last_idle_time_;
};
