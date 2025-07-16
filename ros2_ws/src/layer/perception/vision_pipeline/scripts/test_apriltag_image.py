import cv2
import apriltag
import numpy as np

# ---- 参数设置 ----
image_path = '/home/ros2/ros2_px4/ros2_ws/tmp/tag_01.png'  # 你的 AprilTag 图片路径
tag_size = 0.16  # Tag 实际边长（单位：米）
fx, fy = 600, 600  # 相机焦距（像素）
cx, cy = 320, 240  # 相机光心（像素）

# ---- 加载图像并转灰度 ----
image = cv2.imread(image_path)
if image is None:
    raise FileNotFoundError(f"图像 '{image_path}' 找不到")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# ---- 初始化 detector ----
detector = apriltag.Detector()
detections = detector.detect(gray)

# ---- 遍历每个识别到的 tag ----
for det in detections:
    print(f"[INFO] 检测到 AprilTag ID: {det.tag_id}")
    
    # 定义 tag 世界坐标系角点 (单位: 米)
    half = tag_size / 2
    object_points = np.array([
        [-half, -half, 0],
        [ half, -half, 0],
        [ half,  half, 0],
        [-half,  half, 0]
    ], dtype=np.float32)

    # 图像中的角点
    image_points = np.array(det.corners, dtype=np.float32)

    # 相机内参矩阵
    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0,  0,  1]
    ], dtype=np.float32)
    dist = np.zeros(4)  # 不考虑畸变

    # 求解位姿
    success, rvec, tvec = cv2.solvePnP(object_points, image_points, K, dist)
    if not success:
        print("位姿估计失败")
        continue

    print(f"位移向量 (tvec):\n{tvec}")
    print(f"旋转向量 (rvec):\n{rvec}")

    # 可视化标记
    for corner in det.corners:
        cv2.circle(image, tuple(np.int32(corner)), 5, (0, 255, 0), -1)
    cv2.putText(image, f"ID:{det.tag_id}",
                org=tuple(np.int32(det.center)),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8, color=(0, 0, 255), thickness=2)

# ---- 显示图像 ----
cv2.imshow("AprilTag Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
