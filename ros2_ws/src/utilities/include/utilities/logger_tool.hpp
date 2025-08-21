#ifndef _LOGGER_TOOL_HPP
#define _LOGGER_TOOL_HPP

#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

class LoggerTool {
public:
    LoggerTool(const std::string &log_dir = "~/tmp/ros2_logs/") {
        // 确保目录结尾有 /
        std::string dir = log_dir;
        if (dir.back() != '/') {
            dir += "/";
        }

        // 生成时间戳文件名
        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << dir
            << std::put_time(&tm, "%Y%m%d_%H%M%S")
            << ".log";

        m_log_file.open(oss.str(), std::ios::out | std::ios::app);
        if (!m_log_file.is_open()) {
            throw std::runtime_error("无法打开日志文件: " + oss.str());
        }
    }

    ~LoggerTool() {
        if (m_log_file.is_open()) {
            m_log_file.close();
        }
    }

    template<typename... Args>
    void info(rclcpp::Logger logger, const char *format, Args... args) {
        char buf[1024];
        snprintf(buf, sizeof(buf), format, args...);

        // 输出到 ROS2 控制台
        RCLCPP_INFO(logger, "%s", buf);

        // 写入文件
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_log_file.is_open()) {
            m_log_file << buf << std::endl;
        }
    }

private:
    std::ofstream m_log_file;
    std::mutex m_mutex;
};

#endif
