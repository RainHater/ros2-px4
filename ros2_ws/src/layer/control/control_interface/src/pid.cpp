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
    float integral_limit)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_incr_select = incr_select;
    m_integral_limit = integral_limit;
}

float PIDController::compute(
    float setpoint, 
    float measured_value, 
    float dt)
{
    m_error = setpoint - measured_value;

    if(m_incr_select){  
        float kp_output = (m_kp*(m_error - m_last_error));
        float ki_output = (m_ki * m_error * dt);
        float kd_output = dt?(m_kd * (m_error - m_last_error)) / dt : 0.0f;
        float delta_output = kp_output + ki_output + kd_output;
        
        m_output = m_last_output + delta_output;
        m_last_output = m_output;
        m_prev_error = m_last_error;
        m_last_error = m_error;

        return m_output;

    }else{
        m_integral += m_error * dt;
        m_integral = std::clamp(m_integral, -m_integral_limit, m_integral_limit);

        float derivative = dt?(m_error - m_last_error) / dt:0.0f;

        float kp_output = m_kp * m_error;
        float ki_output = m_ki * m_integral;
        float kd_output = m_kd * derivative;

        m_output = kp_output + ki_output + kd_output;

        m_last_error = m_error;
        return m_output;
    }
}
