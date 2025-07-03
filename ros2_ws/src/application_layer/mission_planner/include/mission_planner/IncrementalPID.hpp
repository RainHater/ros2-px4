#ifndef _INCREMENTAL_PID_H
#define _INCREMENTAL_PID_H

class IncrementalPID {
public:
    IncrementalPID(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd),
          e_k_(0.0f), e_k_1_(0.0f), e_k_2_(0.0f),
          output_(0.0f)
    {}

    // 设置 PID 参数
    void setGains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    // 重置状态
    void reset() {
        e_k_ = e_k_1_ = e_k_2_ = 0.0f;
        output_ = 0.0f;
    }

    // 更新控制器，输入为当前误差，返回增量控制输出
    float update(float error) {
        e_k_2_ = e_k_1_;
        e_k_1_ = e_k_;
        e_k_   = error;

        // 增量式 PID 输出 Δu(k)
        float delta_u = kp_ * (e_k_ - e_k_1_) 
                      + ki_ * e_k_ 
                      + kd_ * (e_k_ - 2.0f * e_k_1_ + e_k_2_);

        // 总输出值（累加增量）
        output_ += delta_u;

        return output_;
    }

    float getOutput() const {
        return output_;
    }

private:
    float kp_, ki_, kd_;
    float e_k_, e_k_1_, e_k_2_; // 当前/前一/前前一误差
    float output_;
};

#endif

