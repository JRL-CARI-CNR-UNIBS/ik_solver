#pragma once
/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */
/**
 * @file logger.h
 * @brief Logging interface used by ik_solver_core.
 *
 * ik_solver_core does not depend on any logging framework. Classes receive a
 * ik_solver::LoggerPtr and write through the IK_* macros. The application
 * decides where messages go by passing its own Logger implementation (e.g. an
 * adapter to rclcpp::Logger, living in the ROS package that uses ik_solver_core).
 * ConsoleLogger is a ready-to-use implementation writing to std::cout/std::cerr.
 */

#include <atomic>
#include <cctype>
#include <chrono>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

namespace ik_solver
{
enum class LogLevel
{
  Trace = 0,
  Debug,
  Info,
  Warn,
  Error,
  Fatal
};

/**
 * @brief Returns the upper-case name of a level ("TRACE", "DEBUG", ...).
 */
inline const char* toString(LogLevel level)
{
  switch (level)
  {
    case LogLevel::Trace:
      return "TRACE";
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Fatal:
      return "FATAL";
  }
  return "UNKNOWN";
}

/**
 * @brief Parses a level name (case-insensitive: trace, debug, info, warn,
 * warning, error, fatal).
 * @return the level, or std::nullopt if the name is not valid.
 */
inline std::optional<LogLevel> logLevelFromString(std::string name)
{
  for (char& c : name)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  if (name == "trace")
    return LogLevel::Trace;
  if (name == "debug")
    return LogLevel::Debug;
  if (name == "info")
    return LogLevel::Info;
  if (name == "warn" || name == "warning")
    return LogLevel::Warn;
  if (name == "error")
    return LogLevel::Error;
  if (name == "fatal")
    return LogLevel::Fatal;
  return std::nullopt;
}

/**
 * @brief Abstract logger.
 *
 * Implementations must be thread safe: the same logger can be shared by
 * objects used from different threads.
 */
class Logger
{
public:
  virtual ~Logger() = default;

  /**
   * @brief Tells if messages of the given level are written. The IK_* macros
   * call it before building the message, so disabled levels cost almost nothing.
   */
  virtual bool enabled(LogLevel level) const = 0;

  /**
   * @brief Writes a message.
   * @param level message level.
   * @param func name of the function that produced the message.
   * @param line source line that produced the message.
   * @param msg the message.
   */
  virtual void log(LogLevel level, const char* func, int line, const std::string& msg) = 0;
};
typedef std::shared_ptr<Logger> LoggerPtr;

/**
 * @brief Logger writing to std::cout (TRACE, DEBUG, INFO) and std::cerr (WARN,
 * ERROR, FATAL), with format "[LEVEL] [name] func, line: msg".
 */
class ConsoleLogger : public Logger
{
protected:
  std::string name_;
  LogLevel min_level_;
  mutable std::mutex mtx_;

public:
  /**
   * @param name name printed in each message.
   * @param min_level messages below this level are discarded.
   */
  explicit ConsoleLogger(const std::string& name = "ik_solver", LogLevel min_level = LogLevel::Info)
    : name_(name), min_level_(min_level)
  {
  }

  void setLevel(LogLevel min_level)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    min_level_ = min_level;
  }

  LogLevel getLevel() const
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return min_level_;
  }

  bool enabled(LogLevel level) const override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    return level >= min_level_;
  }

  void log(LogLevel level, const char* func, int line, const std::string& msg) override
  {
    std::lock_guard<std::mutex> lock(mtx_);
    std::ostream& os = (level >= LogLevel::Warn) ? std::cerr : std::cout;
    os << "[" << toString(level) << "] [" << name_ << "] " << func << ", " << line << ": " << msg << std::endl;
  }
};

/**
 * @brief ANSI color codes, to be streamed into log messages.
 */
namespace color
{
inline std::string RESET()
{
  return "\033[0m";
}
inline std::string BLACK()
{
  return "\033[30m";
}
inline std::string RED()
{
  return "\033[31m";
}
inline std::string GREEN()
{
  return "\033[32m";
}
inline std::string YELLOW()
{
  return "\033[33m";
}
inline std::string BLUE()
{
  return "\033[34m";
}
inline std::string MAGENTA()
{
  return "\033[35m";
}
inline std::string CYAN()
{
  return "\033[36m";
}
inline std::string WHITE()
{
  return "\033[37m";
}
inline std::string BOLDBLACK()
{
  return "\033[1m\033[30m";
}
inline std::string BOLDRED()
{
  return "\033[1m\033[31m";
}
inline std::string BOLDGREEN()
{
  return "\033[1m\033[32m";
}
inline std::string BOLDYELLOW()
{
  return "\033[1m\033[33m";
}
inline std::string BOLDBLUE()
{
  return "\033[1m\033[34m";
}
inline std::string BOLDMAGENTA()
{
  return "\033[1m\033[35m";
}
inline std::string BOLDCYAN()
{
  return "\033[1m\033[36m";
}
inline std::string BOLDWHITE()
{
  return "\033[1m\033[37m";
}
}  // namespace color

}  // namespace ik_solver

/**
 * @brief Writes `stream` (anything that can follow `std::ostream <<`) with the
 * given level. Nothing happens if the logger is null or the level is disabled;
 * in that case `stream` is not evaluated.
 */
#define IK_LOG(logger, level, stream)                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto& ik_logger_ = (logger);                                                                                 \
    if (ik_logger_ && ik_logger_->enabled(level))                                                                      \
    {                                                                                                                  \
      std::ostringstream ik_oss_;                                                                                      \
      ik_oss_ << stream;                                                                                               \
      ik_logger_->log(level, __func__, __LINE__, ik_oss_.str());                                                       \
    }                                                                                                                  \
  } while (0)

#define IK_TRACE(logger, stream) IK_LOG(logger, ik_solver::LogLevel::Trace, stream)
#define IK_DEBUG(logger, stream) IK_LOG(logger, ik_solver::LogLevel::Debug, stream)
#define IK_INFO(logger, stream) IK_LOG(logger, ik_solver::LogLevel::Info, stream)
#define IK_WARN(logger, stream) IK_LOG(logger, ik_solver::LogLevel::Warn, stream)
#define IK_ERROR(logger, stream) IK_LOG(logger, ik_solver::LogLevel::Error, stream)
/// Only logs: it does not abort the program.
#define IK_FATAL(logger, stream) IK_LOG(logger, ik_solver::LogLevel::Fatal, stream)

/**
 * @brief Like IK_LOG, but writes at most once every `period` seconds. The
 * time of the last message is kept per call site (shared by all the objects
 * using that line of code), as in ROS.
 */
#define IK_LOG_THROTTLE(logger, level, period, stream)                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    static std::atomic<int64_t> ik_last_ns_{ std::numeric_limits<int64_t>::min() };                                    \
    const int64_t ik_now_ns_ =                                                                                         \
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())      \
            .count();                                                                                                  \
    const int64_t ik_prev_ns_ = ik_last_ns_.load();                                                                    \
    if (ik_prev_ns_ == std::numeric_limits<int64_t>::min() ||                                                          \
        ik_now_ns_ - ik_prev_ns_ >= static_cast<int64_t>((period) * 1e9))                                              \
    {                                                                                                                  \
      ik_last_ns_.store(ik_now_ns_);                                                                                   \
      IK_LOG(logger, level, stream);                                                                                   \
    }                                                                                                                  \
  } while (0)

#define IK_TRACE_THROTTLE(logger, period, stream) IK_LOG_THROTTLE(logger, ik_solver::LogLevel::Trace, period, stream)
#define IK_DEBUG_THROTTLE(logger, period, stream) IK_LOG_THROTTLE(logger, ik_solver::LogLevel::Debug, period, stream)
#define IK_INFO_THROTTLE(logger, period, stream) IK_LOG_THROTTLE(logger, ik_solver::LogLevel::Info, period, stream)
#define IK_WARN_THROTTLE(logger, period, stream) IK_LOG_THROTTLE(logger, ik_solver::LogLevel::Warn, period, stream)
#define IK_ERROR_THROTTLE(logger, period, stream) IK_LOG_THROTTLE(logger, ik_solver::LogLevel::Error, period, stream)
#define IK_FATAL_THROTTLE(logger, period, stream) IK_LOG_THROTTLE(logger, ik_solver::LogLevel::Fatal, period, stream)
