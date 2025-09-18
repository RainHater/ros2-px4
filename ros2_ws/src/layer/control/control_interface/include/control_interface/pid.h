#ifndef PID_H
#define PID_H

#include <array>

struct PidInfo{
    float kp;
    float ki;
    float kd;
    float th;
};

enum class FilterType {
    None = 0,
    LowPass1,   // 一阶低通
    LowPass2    // 二阶低通 (Butterworth)
};

class PIDController {
public:
    PIDController();

    void reset();

    void initialize(
        float kp, float ki, float kd, 
        bool incr_select, 
        float output_limit,
        float integral_limit = 0.0f,
        float error_filter_tau = 0.0f,               // 30Hz camera  ->  0.05 ~ 0.1
        float derivative_filter_tau = 0.0f,         //  30Hz camera  ->  0.02 ~ 0.05 
        FilterType error_filter_type = FilterType::LowPass1,
        FilterType derivative_filter_type = FilterType::LowPass2
    );
    float compute(float error, float dt);

    inline float lowPassFilter1(float input, float prev, float tau, float dt);

    float lowPassFilter2(float input, std::array<float,2>& x_hist, std::array<float,2>& y_hist, float cutoff, float dt);
    
private:
    // pid 值
    PidInfo m_pid;

    // 误差
    float m_error;
    float m_last_error;
    float m_prev_error;

    // 积分项
    float m_integral;       
    float m_integral_limit;

    float m_output;
    float m_output_limit;
    float m_last_output;

    // 滤波配置
    float m_error_filter_tau;
    float m_derivative_filter_tau;
    FilterType m_error_filter_type;
    FilterType m_derivative_filter_type;

    // 一阶滤波器缓存
    float m_error_filtered;
    float m_derivative_filtered;

    // 二阶滤波器缓存
    std::array<float,2> m_error_x_hist;      
    std::array<float,2> m_error_y_hist;      
    std::array<float,2> m_derivative_x_hist; 
    std::array<float,2> m_derivative_y_hist; 

    // 是否为增量式
    bool m_incr_select;
};

#endif
