import open3d as o3d
import numpy as np
import time
import sys
import os
from scipy.spatial.transform import Rotation as R
import xml.etree.ElementTree as ET

import rospy
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from add_model_gazebo import spawn_model
from add_realsense_gazebo import spawn_realsense
from set_link_state import set_model_state

base_to_depth = np.linalg.inv(np.array([[ 0.00000000e+00, -1.00000000e+00,  0.00000000e+00,  1.75000000e-02],
                          [ 1.11022302e-16, -2.22044605e-16, -1.00000000e+00,  1.25000000e-02],
                          [ 1.00000000e+00,  1.11022302e-16,  0.00000000e+00, -6.93889390e-18],
                          [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]))

nbv_iter = 10

# 从环境变量中获取工作目录
work_dir = os.environ['WORK_DIR']

def quaternion_to_matrix(position, quaternion):
    # 使用scipy库将四元数转换为旋转矩阵
    r = R.from_quat(quaternion)
    rotation_matrix = r.as_matrix()

    # 创建4x4的变换矩阵
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = position

    return T

def matrix_to_quaternion(matrix):
    # 提取旋转矩阵
    rotation_matrix = matrix[:3, :3]

    # 使用scipy库将旋转矩阵转换为四元数
    r = R.from_matrix(rotation_matrix)
    quaternion = r.as_quat()

    # 提取平移向量
    position = matrix[:3, 3]

    return position, quaternion

def pose_to_matrix(pose):
    position = [pose.position.x, pose.position.y, pose.position.z]
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return quaternion_to_matrix(position, quaternion)

def matrix_to_pose(matrix):
    position, quaternion = matrix_to_quaternion(matrix)
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def save_point_cloud(data, camera_pose, path="point_cloud.pcd", voxel_size=0.0001):

    trans = None

    # 将ROS点云消息转换为Open3D点云
    cloud_points = list(pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb")))

    # 创建Open3D点云对象
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector([(p[0], p[1], p[2]) for p in cloud_points])
    
    if len(cloud.points) == 0:
        # 添加一个点，避免空点云
        cloud.points = o3d.utility.Vector3dVector([(0, 0, 0)])
    
    # 处理颜色信息，确保颜色信息是整数类型
    colors = []
    for p in cloud_points:
        rgb = int(p[3])
        r = (rgb >> 16) & 0xFF
        g = (rgb >> 8) & 0xFF
        b = rgb & 0xFF
        colors.append((r / 255.0, g / 255.0, b / 255.0))
    cloud.colors = o3d.utility.Vector3dVector(colors)

    # 把点云转换到世界坐标系
    # camera 是 4*4 的变换矩阵
    trans = camera_pose @ base_to_depth
    cloud.transform(trans)

    # 点云降采样
    cloud = cloud.voxel_down_sample(voxel_size)

    # 给点云上色
    cloud.paint_uniform_color([1, 1, 1])

    # 保存点云到本地文件
    o3d.io.write_point_cloud(path + "/point_cloud.pcd", cloud)
    print("Point cloud saved to %s" % path)

    return trans


def gazebo_pose_to_tf(position, quaternion):
    base_link_matrix = quaternion_to_matrix(position, quaternion)
    depth_link_matrix = base_link_matrix @ base_to_depth
    pose = matrix_to_pose(depth_link_matrix)
    return pose



def parse_pointcloud2(msg):
    # 解析 PointCloud2 消息
    points = pc2.read_points(msg, field_names=("x", "y", "z", "view_x", "view_y", "view_z"), skip_nans=True)
    
    for point in points:
        x, y, z, view_x, view_y, view_z = point

        # print("x: %f, y: %f, z: %f, view_x: %f, view_y: %f, view_z: %f" % (x, y, z, view_x, view_y, view_z))

        # 构建齐次变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[0, 3] = x
        transform_matrix[1, 3] = y
        transform_matrix[2, 3] = z

        # view_x, view_y, view_z 是x轴的方向向量
        z_axis = np.array([view_x, view_y, view_z])
        z_axis /= np.linalg.norm(z_axis)

        # 生成x轴和y轴
        x_axis = np.cross([0, 0, -1], z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        # 生成齐次变换矩阵
        transform_matrix[:3, 0] = x_axis
        transform_matrix[:3, 1] = y_axis
        transform_matrix[:3, 2] = z_axis

    return transform_matrix


def read_rho_from_launch(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    rho_value = None

    # 查找 rho 参数
    for include in root.findall('include'):
        for arg in include.findall('arg'):
            if arg.get('name') == 'rho':
                rho_value = float(arg.get('value'))
                print(f"Found rho value: {rho_value}")
                break

    if rho_value is None:
        print("rho parameter not found in the launch file.")
    return rho_value


def modify_tho_launch_file(file_path, data):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # 查找并修改 rho 参数
    for include in root.findall('include'):
        for arg in include.findall('arg'):
            if arg.get('name') == 'rho':
                current_value = float(arg.get('value'))
                new_value = data
                arg.set('value', str(new_value))
                print(f"Modified rho value from {current_value} to {new_value}")

    # 保存修改后的文件
    tree.write(file_path)
    print(f"Saved modified launch file to {file_path}")


if __name__ == "__main__":
    launch_file_path = f'{work_dir}src/see_core/launch/run_see.launch'
    data = read_rho_from_launch(launch_file_path)
    print(data)