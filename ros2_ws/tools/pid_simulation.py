import re
import yaml
import argparse
from pid import PIDController

def get_pattern():
    pattern = r"(\w+):\s*([-+]?\d+(?:\.\d+)?)"
    return pattern

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="读取日志并处理")
    parser.add_argument('-t', '--target', help='要读取的文件路径', default='tools/logs/20000101_080306.log')
    args = parser.parse_args()

    with open("src/utilities/config/app.yaml", "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    yaw_kp = config['track']['yaw_kp']
    yaw_ki = config['track']['yaw_ki']
    yaw_kd = config['track']['yaw_kd']
    ud_kp = config['track']['ud_kp']
    ud_ki = config['track']['ud_ki']
    ud_kd = config['track']['ud_kd']
    fb_kp = config['track']['fb_kp']
    fb_ki = config['track']['fb_ki']
    fb_kd = config['track']['fb_kd']

    yaw_pid = PIDController()
    ud_pid = PIDController()
    fb_pid = PIDController()

    yaw_pid.initialize(yaw_kp, yaw_ki, yaw_kd, False, 0.785, 0.785)
    ud_pid.initialize(ud_kp, ud_ki, ud_kd, False, 0.5, 0.5)
    fb_pid.initialize(fb_kp, fb_ki, fb_kd, False, 0.5, 0.5)

    pattern = get_pattern()

    rows = []
    with open(args.target, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            matches = re.findall(pattern, line)
            row = {k: float(v) for k, v in matches}
            rows.append(row)
    print(f'rows: {rows}')

    for row in rows:
        cx = row['cx']
        cx_error = row['cx_error']
        cx_out = row['cx_out']
        cy = row['cy']
        cy_error = row['cy_error']
        cy_out = row['cy_out']
        dt = row['dt']
        area_speed = row['area_speed']
        area = row['area']
        
        error = 864000.0 - area
        output = fb_pid.compute(error, dt)
        print(f'area: {area}, output: {output}')
    
    # for mat in matches:
    #     pixel_offset = float(mat[0]) - 1920.0 / 2.0
    #     angle_offset = (pixel_offset)
    #     # angle_offset = pixel_offset
    #     setpoint = angle_offset
    #     out = yaw_pid.compute(setpoint)
    #     print(f'cx: {mat[0]:<8} setpoint: {setpoint:>10}, out: {out}')
