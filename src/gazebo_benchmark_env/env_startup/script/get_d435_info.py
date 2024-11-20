import rospy
import math
from sensor_msgs.msg import CameraInfo

def calculate_fov(focal_length, sensor_size):
    """
    计算视场角
    
    :param focal_length: 焦距（单位：像素）
    :param sensor_size: 传感器尺寸（单位：像素）
    :return: 视场角（单位：度）
    """
    fov = 2 * math.atan(sensor_size / (2 * focal_length))
    return math.degrees(fov)

def camera_info_callback(msg):
    # 处理接收到的相机信息
    print("Received camera info:")
    print(msg)

    # 假设传感器尺寸与图像尺寸相同
    sensor_width = msg.width
    sensor_height = msg.height

    # 提取相机内参
    print("Extracting pb camera intrinsics...")
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]


    fov_x = calculate_fov(fx, sensor_width)
    fov_y = calculate_fov(fy, sensor_height)

    print("Camera Intrinsics:")
    print(f"fx: {fx}")
    print(f"fy: {fy}")
    print(f"cx: {cx}")
    print(f"cy: {cy}")
    # 合成相机内参矩阵
    K = [[fx, 0, cx],
         [0, fy, cy],
         [0, 0, 1]]
    print("Camera Intrinsics Matrix:")
    print(K)
    print(f"水平视场角 FOV_x: {fov_x:.2f} 度")
    print(f"垂直视场角 FOV_y: {fov_y:.2f} 度")

if __name__ == "__main__":
    rospy.init_node('get_d435_info')

    # 等待从话题 '/d435/depth/camera_info' 接收到一帧消息
    print("Waiting for camera info message...")
    camera_info_msg = rospy.wait_for_message('/d435/depth/camera_info', CameraInfo)
    
    # 调用回调函数处理接收到的消息
    camera_info_callback(camera_info_msg)