float32 pixel_dist    # 像素空间中的距离（目标中心到图像中心）
float32 angle_x       # 水平偏差角，右偏为正（角度制）
float32 angle_y       # 垂直偏差角，下偏为正（角度制）
float32 angle         # 视线方向综合角度，可用于偏航控制
bool    target_found  # 判断是否识别到目标