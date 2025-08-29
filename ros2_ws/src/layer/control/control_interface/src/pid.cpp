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
}

void PIDController::initialize(
    float kp, float ki, float kd, 
    bool incr_select, 
    float output_limit,
    float integral_limit)
{   
    m_pid.kp = kp;
    m_pid.ki = ki;
    m_pid.kd = kd;
    m_incr_select = incr_select;
    m_output_limit = output_limit;
    m_integral_limit = integral_limit;
}

float PIDController::compute(float error, float dt) {
    m_error = error;

    if(m_incr_select){  
        float kp_output = (m_pid.kp*(m_error - m_last_error));
        float ki_output = (m_pid.ki * m_error * dt);
        float kd_output = dt?(m_pid.kd * (m_error - m_last_error)) / dt : 0.0f;
        float delta_output = kp_output + ki_output + kd_output;
        
        m_output = m_last_output + delta_output;
        m_output = std::clamp(m_output, -m_output_limit, m_output_limit);
        m_last_output = m_output;
        m_prev_error = m_last_error;
        m_last_error = m_error;

        return m_output;

    }else{
        m_integral += m_error * dt;
        m_integral = std::clamp(m_integral, -m_integral_limit, m_integral_limit);

        float derivative = dt?(m_error - m_last_error) / dt:0.0f;

        float kp_output = m_pid.kp * m_error;
        float ki_output = m_pid.ki * m_integral;
        float kd_output = m_pid.kd * derivative;

        m_output = kp_output + ki_output + kd_output;
        m_output = std::clamp(m_output, -m_output_limit, m_output_limit);

        m_last_error = m_error;

        return m_output;
    }
}
