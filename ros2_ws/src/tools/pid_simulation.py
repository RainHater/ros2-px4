import re
import yaml
import argparse
from pid import PIDController

def get_pattern():
    pattern = r"cx:\s*(\d+),.*?yawspeed:\s*([-]?\d+\.\d+)"

    return pattern

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="读取日志并处理")
    parser.add_argument('-t', '--target', help='要读取的文件路径', default='tmp/log/2.txt')
    args = parser.parse_args()

    with open("ros2_ws/src/utilities/config/app.yaml", "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    cx_kp = config['track']['cx_kp']
    cx_ki = config['track']['cx_ki']
    cx_kd = config['track']['cx_kd']
    cy_kp = config['track']['cy_kp']
    cy_ki = config['track']['cy_ki']
    cy_kd = config['track']['cy_kd']

    cx_pid = PIDController()
    cy_pid = PIDController()

    cx_pid.initialize(cx_kp, cx_ki, cx_kd, False, 0.785, 0.785)
    cy_pid.initialize(cy_kp, cy_ki, cy_kd, False, 0.5, 0.5)

    pattern = get_pattern()

    with open(args.target, 'r', encoding='utf-8') as f:
        data = f.read()
        matches = re.findall(pattern, data)
    
    for mat in matches:
        pixel_offset = float(mat[0]) - 1920.0 / 2.0
        angle_offset = (pixel_offset)
        # angle_offset = pixel_offset
        setpoint = angle_offset
        out = cx_pid.compute(setpoint)
        print(f'cx: {mat[0]:<8} setpoint: {setpoint:>10}, out: {out}')
