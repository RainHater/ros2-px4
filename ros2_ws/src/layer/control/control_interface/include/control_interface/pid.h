#ifndef PID_H
#define PID_H

class PIDController {
public:
    PIDController();

    void reset();

    void initialize(
        float kp, float ki, float kd, 
        bool incr_select, 
        float output_limit,
        float integral_limit = 0.0f
    );
    float compute(float error, float dt);
private:
    //比例增益
    float m_kp;
    //积分增益           
    float m_ki;
    //微分增益             
    float m_kd;

    //误差
    float m_error;
    //上次误差
    float m_last_error;
    float m_prev_error;

    //积分项
    float m_integral;       
    //积分饱和限制
    float m_integral_limit;

    float m_output;
    float m_output_limit;
    float m_last_output;

    //是否为增量式
    bool m_incr_select;
};

#endif
