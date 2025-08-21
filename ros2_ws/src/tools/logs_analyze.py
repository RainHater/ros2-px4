import os
import re
import csv
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# 输入日志文件路径
log_file = "ros2_ws/src/tools/track.log"
plt_out_path = "ros2_ws/src/tools/output/track"

# 基准线（可以自定义）
baseline_cx = 0.0   # cx_error 基准
baseline_cy = 0.0   # cy_error 基准

# 正则模式
pattern = re.compile(
    r"cx:\s*([-+]?\d*\.?\d+),\s*"
    r"cx_error:\s*([-+]?\d*\.?\d+),\s*"
    r"cx_out:\s*([-+]?\d*\.?\d+),\s*"
    r"cy:\s*([-+]?\d*\.?\d+),\s*"
    r"cy_error:\s*([-+]?\d*\.?\d+),\s*"
    r"cy_out:\s*([-+]?\d*\.?\d+),\s*"
    r"dt:\s*([-+]?\d*\.?\d+)"
)

def csv_write(out_folder, name, col_l: list, data_l: list):
    with open(f"{out_folder}/{name}.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(col_l)
        for cx_l in data_l:
            writer.writerow(cx_l)

def plt_write(out_folder, name, data_l: list):
    if data_l:
        data = np.array(data_l)
        x = np.cumsum(data[:, 3])
        plt.figure(figsize=(10, 5))
        plt.plot(x, data[:, 1], label=name, color="blue")
        plt.axhline(y=baseline_cx, color="red", linestyle="--", label=f"Baseline={baseline_cx}")
        plt.xlabel("Sample Index")
        plt.ylabel(f"{name} Value")
        plt.title(f"{name} over Time")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(f"{out_folder}/{name}.png")
        plt.close()
        print(f"📊 已保存: {out_folder}/{name}.png")

# 读取日志
cx_list = []
cy_list = []
with open(log_file, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            values = list(map(float, match.groups()))
            cx_list.append([values[0], values[1], values[2], values[6]])
            cy_list.append([values[3], values[4], values[5], values[6]])
 
# 如果有数据就画图
timestamp = datetime.now().strftime("%Y-%m-%d_%H%M")
out_folder = f"{plt_out_path}/{timestamp}"
os.makedirs(out_folder, exist_ok=True)

# 保存到 CSV
csv_write(out_folder, 'cx', ["cx", "cx_error", "cx_out", "dt"], cx_list)
csv_write(out_folder, 'cy', ["cy", "cy_error", "cy_out", "dt"], cy_list)

print(f"✅ 提取完成, csv已保存到 {out_folder}")

plt_write(out_folder, 'cx_error', cx_list)
plt_write(out_folder, 'cy_error', cy_list)
