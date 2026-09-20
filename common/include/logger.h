// logger.h - Lightweight leveled logger
#pragma once

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace rm_radar {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger {
public:
    static Logger& instance() { static Logger l; return l; }
    void setLevel(LogLevel l) { level_ = l; }

    void log(LogLevel l, const std::string& msg, const char* file, int line) {
        if (l < level_) return;
        const char* tags[] = {"DEBUG", "INFO ", "WARN ", "ERROR"};
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) % 1000;
        std::tm tm;
        localtime_r(&t, &tm);
        std::ostream& out = (l >= LogLevel::WARN) ? std::cerr : std::cout;
        out << "[" << std::put_time(&tm, "%H:%M:%S") << "."
            << std::setfill('0') << std::setw(3) << ms.count() << "] "
            << "[" << tags[static_cast<int>(l)] << "] " << msg
            << "  (" << file << ":" << line << ")\n";
    }

private:
    LogLevel level_ = LogLevel::INFO;
};

#define RM_LOG_DEBUG(msg) rm_radar::Logger::instance().log(rm_radar::LogLevel::DEBUG, msg, __FILE__, __LINE__)
#define RM_LOG_INFO(msg)  rm_radar::Logger::instance().log(rm_radar::LogLevel::INFO,  msg, __FILE__, __LINE__)
#define RM_LOG_WARN(msg)  rm_radar::Logger::instance().log(rm_radar::LogLevel::WARN,  msg, __FILE__, __LINE__)
#define RM_LOG_ERROR(msg) rm_radar::Logger::instance().log(rm_radar::LogLevel::ERROR, msg, __FILE__, __LINE__)

} // namespace rm_radar
