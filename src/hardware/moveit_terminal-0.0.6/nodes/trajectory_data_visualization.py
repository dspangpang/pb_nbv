#! /usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
import rospy
import time 

# 读取文件夹下的所以文件
def read_dir_file(dir_path):
    file_list = os.listdir(dir_path)
    for i in range(len(file_list)):
        file_list[i] = dir_path + '/' + file_list[i]
    return file_list
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
# 读取txt文件
def read_txt_file(file_path):
    data = []
    with open(file_path, 'r') as f:
        for line in f.readlines():
            line = line.strip('\n')
            line = line.split(' ')
            data.append(line)
            
    return data 

# 显示文件
def show_txt_file(file_path):
    data = read_txt_file(file_path)
    for i in range(len(data)):
        print(data[i])

# 把数据转换成位姿数据
def data_to_pose(data):
    # 位置数据 x y z
    pose = []
    # 姿态数据 w x y z
    quat = []
    for i in range(len(data)):
        pose.append([float(data[i][0]), float(data[i][1]), float(data[i][2])])
        quat.append([float(data[i][3]), float(data[i][4]), float(data[i][5]), float(data[i][6])])
    return pose, quat

# 把数据转换成速度数据
def data_to_velocity(data):
    # 速度数据 x y z
    velocity = []
    for i in range(len(data)):
        tmp = np.sqrt(float(data[i][0])**2 + float(data[i][1])**2 + float(data[i][2])**2)
        velocity.append([float(data[i][0]), float(data[i][1]), float(data[i][2]), float(tmp)])
    return velocity

# 可视化一列数据
def show_velocity_data(data):

    global fig
    global ax

    # 从data中提取四条数据
    data_v = data[:, 3]
    data_z_v = data[:, 2]
    data_y_v = data[:, 1]
    data_x_v = data[:, 0]

    # 画图
    plt.subplot(122)  
    plt.plot(data_v, label='liner_velocity')
    plt.plot(data_x_v, label='liner_x_velocity')
    plt.plot(data_y_v, label='liner_y_velocity')
    plt.plot(data_z_v, label='liner_z_velocity')

    # 添加文本标签 保留两位小数
    for i in range(len(data_v)):
        plt.text(i, data_v[i], "{:.2f}".format(data_v[i]))
    for i in range(len(data_x_v)):
        plt.text(i, data_x_v[i], "{:.2f}".format(data_x_v[i]))
    for i in range(len(data_y_v)):
        plt.text(i, data_y_v[i], "{:.2f}".format(data_y_v[i]))
    for i in range(len(data_z_v)):
        plt.text(i, data_z_v[i], "{:.2f}".format(data_z_v[i]))



    # 设置坐标轴
    plt.xlabel('x')
    plt.ylabel('y')
    # 显示图像
    plt.legend()


# 显示3D离散数据
def show_pose_data(pose, quat):

    global fig
    global ax
    
    # 创建坐标系的三个轴
    # x轴，y轴，z轴
    axes = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    ax = fig.add_subplot(121, projection='3d')
    
    # 根据四元数画坐标系
    for i in range(len(quat)):

        r = R.from_quat(quat[i])
        # 通过四元数旋转坐标轴
        rotated_axes = r.apply(axes)

        scale_factor = 0.01

        # 旋转后的坐标轴
        
        ax.quiver(pose[i][0], pose[i][1], pose[i][2],
                  rotated_axes[0, 0], rotated_axes[1, 0], rotated_axes[2, 0], 
                  color='r', label='X\'', length=scale_factor)
        ax.quiver(pose[i][0], pose[i][1], pose[i][2],
                    rotated_axes[0, 1], rotated_axes[1, 1], rotated_axes[2, 1], 
                    color='g', label='Y\'', length=scale_factor)
        ax.quiver(pose[i][0], pose[i][1], pose[i][2],
                    rotated_axes[0, 2], rotated_axes[1, 2], rotated_axes[2, 2], 
                    color='b', label='Z\'', length=scale_factor)
        
    # 设置坐标轴
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    # 计算数据点的范围
    x_min, x_max = np.min(pose[:, 0]), np.max(pose[:, 0])
    y_min, y_max = np.min(pose[:, 1]), np.max(pose[:, 1])
    z_min, z_max = np.min(pose[:, 2]), np.max(pose[:, 2])
    data_range = max(x_max - x_min, y_max - y_min, z_max - z_min)

    # 设置坐标轴范围相同
    ax.set_xlim(x_min, x_min + data_range)
    ax.set_ylim(y_min, y_min + data_range)
    ax.set_zlim(z_min, z_min + data_range)
    



# 主函数
def main(path):
    # 读取文件
    global fig
    global ax

    dir_path = path

    file_list = read_dir_file(dir_path)
    pose_data = read_txt_file(file_list[1])
    velocity_data = read_txt_file(file_list[0])

    # 文件处理
    pose, quat = data_to_pose(pose_data)
    velocity = data_to_velocity(velocity_data)

    show_pose_data(np.array(pose), np.array(quat))
    show_velocity_data(np.array(velocity))

    ax.relim()
    ax.autoscale_view()
    plt.draw()

    # 暂停一段时间，等待新数据
    plt.pause(0.01)
    


# 主函数
def main(path):
    # 读取文件
    global fig
    global ax

    dir_path = path

    file_list = read_dir_file(dir_path)
    pose_data = read_txt_file(file_list[1])
    velocity_data = read_txt_file(file_list[0])

    # 文件处理
    pose, quat = data_to_pose(pose_data)
    velocity = data_to_velocity(velocity_data)

    show_pose_data(np.array(pose), np.array(quat))
    show_velocity_data(np.array(velocity))

    ax.relim()
    ax.autoscale_view()
    plt.draw()

    # 暂停一段时间，等待新数据
    plt.pause(0.01)


# 主函数
if __name__ == '__main__':

    # 创建画布
    global fig
    global ax

    fig, ax = plt.subplots(figsize=(16, 8), num='trajectory_data_visualization_tool')

    # 初始化ros
    rospy.init_node('trajectory_data_visualization', anonymous=True)

    # 获取参数
    node_name = rospy.get_name()    
    folder_path = rospy.get_param(node_name + '/folder_path')
    print(folder_path)

    # 创建一个初始文件列表快照
    initial_folders = set(os.listdir(folder_path))

    while rospy.is_shutdown() == False:
        # 暂停一段时间，可以根据需要调整检测频率
        time.sleep(1)  # 60秒检查一次

        # 获取当前文件夹列表
        current_folders = set(os.listdir(folder_path))

        # 检查新增文件夹
        added_folders = current_folders - initial_folders

        if added_folders:
            plt.cla()
            for folder in added_folders:
                main(folder_path + '/' + folder)

        # 更新文件夹列表快照
        initial_folders = current_folders

    plt.close()