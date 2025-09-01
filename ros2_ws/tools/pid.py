class PIDController:
    def __init__(self):
        self.reset()
        self.m_kp = 0.0
        self.m_ki = 0.0
        self.m_kd = 0.0
        self.m_incr_select = False
        self.m_output_limit = 0.0
        self.m_integral_limit = 0.0

    def reset(self):
        self.m_error = 0.0
        self.m_last_error = 0.0
        self.m_prev_error = 0.0
        self.m_integral = 0.0
        self.m_output = 0.0
        self.m_last_output = 0.0

    def initialize(self, kp, ki, kd, incr_select, output_limit=0.0, integral_limit=0.0):
        self.m_kp = kp
        self.m_ki = ki
        self.m_kd = kd
        self.m_incr_select = incr_select
        self.m_output_limit = output_limit
        self.m_integral_limit = integral_limit

    def compute(self, error, dt=1.0 / 30.0):
        self.m_error = error

        if self.m_incr_select:
            # 增量式 PID
            kp_output = self.m_kp * (self.m_error - self.m_last_error)
            ki_output = self.m_ki * self.m_error * dt
            kd_output = (self.m_kd * (self.m_error - self.m_last_error) / dt) if dt != 0 else 0.0

            delta_output = kp_output + ki_output + kd_output
            self.m_output = self.m_last_output + delta_output

            # 输出限幅
            self.m_output = max(min(self.m_output, self.m_output_limit), -self.m_output_limit)

            self.m_last_output = self.m_output
            self.m_prev_error = self.m_last_error
            self.m_last_error = self.m_error
            return self.m_output

        else:
            # 位置式 PID
            self.m_integral += self.m_error * dt
            # 限制积分
            self.m_integral = max(min(self.m_integral, self.m_integral_limit), -self.m_integral_limit)

            derivative = (self.m_error - self.m_last_error) / dt if dt != 0 else 0.0

            kp_output = self.m_kp * self.m_error
            ki_output = self.m_ki * self.m_integral
            kd_output = self.m_kd * derivative

            self.m_output = kp_output + ki_output + kd_output
            print(f'output: {self.m_output}')
            # 输出限幅
            self.m_output = max(min(self.m_output, self.m_output_limit), -self.m_output_limit)

            self.m_last_error = self.m_error
            return self.m_output
