/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

/**
 * @file SafeQueue.h
 * @brief Mutex-protected queue.
 */
#ifndef IK_SOLVER__INTERNAL__SAFE_QUEUE_H
#define IK_SOLVER__INTERNAL__SAFE_QUEUE_H

#include <mutex>
#include <queue>

namespace ik_solver
{
// Thread safe implementation of a Queue using an std::queue
template <typename T>
/**
 * @brief Thread-safe FIFO queue based on std::queue and a mutex. Used to distribute IK/FK jobs to worker threads.
 */
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

  /**
   * @brief True if the queue is empty.
   */
  bool empty()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  /**
   * @brief Number of elements.
   */
  int size()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  }

  /**
   * @brief Push an element at the back.
   */
  void enqueue(T& t)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    queue_.push(t);
  }

  /**
   * @brief Pop the front element.
   * @param[out] t Popped element.
   * @return False if the queue is empty.
   */
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