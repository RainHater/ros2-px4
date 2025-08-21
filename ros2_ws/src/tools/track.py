import random

# 输出日志文件
log_file = "ros2_ws/src/tools/track.log"

# 固定采样周期
dt_value = round(1/30, 5)  # 30Hz采样，每条0.03333秒

# 生成 200 行模拟数据
with open(log_file, "w") as f:
    for i in range(200):
        cx = round(random.uniform(100, 200), 3)
        cx_error = round(random.uniform(-5, 5), 3)
        cx_out = round(random.uniform(-1, 1), 3)
        cy = round(random.uniform(300, 400), 3)
        cy_error = round(random.uniform(-5, 5), 3)
        cy_out = round(random.uniform(-1, 1), 3)
        dt = dt_value  # 固定每条采样间隔

        line = (
            f"cx: {cx}, cx_error: {cx_error}, cx_out: {cx_out}, "
            f"cy: {cy}, cy_error: {cy_error}, cy_out: {cy_out}, dt: {dt}\n"
        )
        f.write(line)

print(f"✅ 已生成固定采样周期日志文件: {log_file}")
