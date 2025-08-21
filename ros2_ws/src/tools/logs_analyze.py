import re

# 你日志文件路径
log_file = "track.log"
# 输出 CSV 文件
csv_file = "track_parsed.csv"

# 正则表达式（和你的一样）
pattern = re.compile(
    r"cx:\s*([-+]?\d*\.?\d+),\s*"
    r"cx_error:\s*([-+]?\d*\.?\d+),\s*"
    r"cx_out:\s*([-+]?\d*\.?\d+),\s*"
    r"cy:\s*([-+]?\d*\.?\d+),\s*"
    r"cy_error:\s*([-+]?\d*\.?\d+),\s*"
    r"cy_out:\s*([-+]?\d*\.?\d+)"
)

# 打开日志，逐行读取
results = []
with open(log_file, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            values = list(map(float, match.groups()))
            results.append(values)

print(f"results: {results}")
