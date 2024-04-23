/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#ifndef IK_SOLVER__INTERNAL__SAFE_QUEUE_H
#define IK_SOLVER__INTERNAL__SAFE_QUEUE_H

#include <mutex>
#include <queue>

namespace ik_solver
{
// Thread safe implementation of a Queue using an std::queue
template <typename T>
class SafeQueue
{
private:
  std::queue<T> queue_;
  std::mutex mutex_;

public:
  SafeQueue()
  {
  }

  SafeQueue(SafeQueue& other)
  {
    // TODO:
  }

  ~SafeQueue()
  {
  }

  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  int size()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  }

  void enqueue(T& t)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(t);
  }

  bool dequeue(T& t)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty())
    {
      return false;
    }
    t = std::move(queue_.front());

    queue_.pop();
    return true;
  }
};

}  // namespace thread_pool

#endif