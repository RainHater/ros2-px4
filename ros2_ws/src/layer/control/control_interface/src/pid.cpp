#include "control_interface/pid.h"
#include <algorithm>

PIDController::PIDController() {
    reset();
}

void PIDController::reset(){
    m_error = 0.0f;
    m_last_error = 0.0f;
    m_prev_error = 0.0f;

    m_integral = 0.0f;
    m_integral_limit = 0.0;

    m_output = 0.0f;
    m_last_output = 0.0f;

    m_error_filtered = 0.0f;
    m_derivative_filtered = 0.0f;

    m_error_x_hist = {0.0f, 0.0f};
    m_error_y_hist = {0.0f, 0.0f};
    m_derivative_x_hist = {0.0f, 0.0f};
    m_derivative_y_hist = {0.0f, 0.0f};
}

void PIDController::initialize(
    float kp, float ki, float kd, 
    bool incr_select, 
    float output_limit,
    float integral_limit,
    float error_filter_tau,      // 误差滤波常数
    float derivative_filter_tau,  // 微分滤波常数
    FilterType error_filter_type,
    FilterType derivative_filter_type
)
{   
    m_pid.kp = kp;
    m_pid.ki = ki;
    m_pid.kd = kd;
    m_incr_select = incr_select;
    m_output_limit = output_limit;
    m_integral_limit = integral_limit;

    // filter coefficient
    m_error_filter_tau = error_filter_tau;
    m_derivative_filter_tau = derivative_filter_tau;
    
    // type
    m_error_filter_type = error_filter_type;
    m_derivative_filter_type = derivative_filter_type;
}

// 一阶IIR  y = α*x + (1-α)*y_last = α(x-y_last) + y_last  
inline float PIDController::lowPassFilter1(float input, float prev, float tau, float dt){
    if(tau <= 1e-6f) return input;
    float alpha = dt / (tau + dt);
    return prev + alpha * (input - prev);
}

// 二阶IIR y[n]=b0​x[n]+b1​x[n−1]+b2​x[n−2]−a1​y[n−1]−a2​y[n−2]
float PIDController::lowPassFilter2(
    float input,
    std::array<float,2>& x_hist,
    std::array<float,2>& y_hist,
    float tau, 
    float dt
) 
{
    if(tau <= 1e-6f) return input;

    // cutoff = 1/(2π tau)
    float cutoff = 1.0f / (2.0f * M_PI * tau);
    float omega = 2.0f * M_PI * cutoff * dt;
    float cos_omega = cosf(omega);
    float sin_omega = sinf(omega);

    float q = 0.7071f; // Butterworth Q值 = 1/sqrt(2)
    float alpha = sin_omega / (2.0f * q);

    float b0 = (1.0f - cos_omega) / 2.0f;
    float b1 = 1.0f - cos_omega;
    float b2 = (1.0f - cos_omega) / 2.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_omega;
    float a2 = 1.0f - alpha;

    // 归一化
    b0 /= a0; b1 /= a0; b2 /= a0;
    a1 /= a0; a2 /= a0;

    // IIR 双二阶实现
    float y = b0 * input + b1 * x_hist[0] + b2 * x_hist[1]
    - a1 * y_hist[0] - a2 * y_hist[1];

    // 更新历史值
    x_hist[1] = x_hist[0];
    x_hist[0] = input;
    y_hist[1] = y_hist[0];
    y_hist[0] = y;

    return y;
}

float PIDController::compute(float error, float dt) {
    switch(m_error_filter_type) {
        case FilterType::LowPass1:
            m_error = lowPassFilter1(error, m_error_filtered, m_error_filter_tau, dt);
            m_error_filtered = m_error;
            break;
        case FilterType::LowPass2:
            m_error = lowPassFilter2(error, m_error_x_hist, m_error_y_hist, m_error_filter_tau, dt);
            break;
        default:
            m_error = error;
            break;
    }

    if(m_incr_select){  
        float kp_output = m_pid.kp * (m_error - m_last_error);
        float ki_output = m_pid.ki * m_error * dt;

        float raw_derivative = dt ? (m_error - m_last_error) / dt : 0.0f;
        float derivative = 0.0f;
        switch(m_derivative_filter_type) {
            case FilterType::LowPass1:
                derivative = lowPassFilter1(raw_derivative, m_derivative_filtered, m_derivative_filter_tau, dt);
                m_derivative_filtered = derivative;
                break;
            case FilterType::LowPass2:
                derivative = lowPassFilter2(raw_derivative, m_derivative_x_hist, m_derivative_y_hist, m_derivative_filter_tau, dt);
                break;
            default:
                derivative = raw_derivative;
                break;
        }

        float kd_output = m_pid.kd * derivative;
        float delta_output = kp_output + ki_output + kd_output;
        
        m_output = m_last_output + delta_output;
        m_output = std::clamp(m_output, -m_output_limit, m_output_limit);

        m_last_output = m_output;
        m_prev_error = m_last_error;
        m_last_error = m_error;

        return m_output;

    }else{
        // 积分
        m_integral += m_error * dt;
        m_integral = std::clamp(m_integral, -m_integral_limit, m_integral_limit);

        // 微分滤波
        float raw_derivative = dt ? (m_error - m_last_error) / dt : 0.0f;
        float derivative = 0.0f;
        switch(m_derivative_filter_type) {
            case FilterType::LowPass1:
                derivative = lowPassFilter1(raw_derivative, m_derivative_filtered, m_derivative_filter_tau, dt);
                m_derivative_filtered = derivative;
                break;
            case FilterType::LowPass2:
                derivative = lowPassFilter2(raw_derivative, m_derivative_x_hist, m_derivative_y_hist, m_derivative_filter_tau, dt);
                break;
            default:
                derivative = raw_derivative;
                break;
        }

        float kp_output = m_pid.kp * m_error;
        float ki_output = m_pid.ki * m_integral;
        float kd_output = m_pid.kd * derivative;

        m_output = kp_output + ki_output + kd_output;
        m_output = std::clamp(m_output, -m_output_limit, m_output_limit);

        m_last_error = m_error;

        return m_output;
    }
}