#include <stdint.h>

class TaskNode {
public:
  typedef void (*Callback)(uint32_t);

  uint32_t id() const { return id_; }
  void run(uint32_t timeNow) const {
    if (cb_) {
      cb_(timeNow);
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
  Callback cb_;
};

class TaskManager {
public:    
  TaskManager() {
    reset();
  }

  void reset() {
    task_list_start_ = task_list_end_ = nullptr;
    for(auto& node: pool_) {
      node.in_use_ = false;
    }
  }

  TaskNode* newTask(
      uint32_t id,
      uint32_t timeNow,
      uint32_t interval,
      TaskNode::Callback cb) {
    for(auto& node : pool_) {
      if (!node.in_use_) {
        node.in_use_ = true;
        node.id_ = id;
        node.sched_time_ = timeNow;
        node.interval_ = interval;
        node.cb_ = cb;
        node.prev_ = nullptr;
        node.next_ = nullptr;
        return &node;
      }
    }
    HandleError("Task pool is full");
    return nullptr;
  }

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

  void remove(TaskNode* node) {
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

  TaskNode* findById(uint32_t id) {
    return findFirst([id](const TaskNode* node) {
      return node->id_ == id;
    });
  }

  TaskNode* findNext(uint32_t time) {
    return findFirst([time](const TaskNode* node) {
      return ((time - node->sched_time_) >= node->interval_);
    });
  }
private:
  void HandleError(const char*) {
  }
 
  TaskNode pool_[32];
  TaskNode *task_list_start_;
  TaskNode *task_list_end_;
};