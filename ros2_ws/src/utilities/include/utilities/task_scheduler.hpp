#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <vector>
#include <memory>

namespace task_tool {

struct Task {
    std::function<void(std::shared_ptr<bool> &)> start;
    std::shared_ptr<bool> finished_flag;
    bool started = false;

    bool is_finished() const {
        return finished_flag && *finished_flag;
    }
};

class TaskScheduler {
public:
    // 添加任务时返回控制该任务完成标志位的 shared_ptr
    void add_task(std::function<void(std::shared_ptr<bool> &)> start_fn) {
        auto flag = std::make_shared<bool>(false);
        m_tasks.push_back(Task{start_fn, flag});
    }

    void run() {
        if (m_current_index >= m_tasks.size()) return;

        Task& current = m_tasks[m_current_index];
        if (!current.started) {
            current.start(current.finished_flag);
            current.started = true;
        }

        if (current.is_finished()) {
            m_current_index++;
        }
    }

    bool is_done() const {
        return m_current_index >= m_tasks.size();
    }

private:
    std::vector<Task> m_tasks;
    size_t m_current_index = 0;
};

}
