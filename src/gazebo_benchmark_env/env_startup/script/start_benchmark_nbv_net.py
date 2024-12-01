import open3d as o3d
import numpy as np
import time
import sys
import os
import subprocess

import rospy
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from utils_msgs.srv import NBVTrigger, NBVTriggerRequest
from std_msgs.msg import String

from add_model_gazebo import spawn_model, delete_model
from add_realsense_gazebo import spawn_realsense
from set_link_state import set_model_state
from utils import quaternion_to_matrix, save_point_cloud, matrix_to_quaternion, pose_to_matrix, base_to_depth, nbv_iter

model_dir = ["hb_models", "lm_models", "stanford_models"]
# model_dir = ["hb_models"]
method_type = ["nbvnet"]
scvp_config_file = "/root/work_place/pb_nbv/src/scvp_core/config/DefaultConfiguration.yaml"

# 由于scvp的坐标系和gazebo相机的坐标系不同，所以需要一个矩阵进行转换
camera_diff_matrix = np.array([[  0.0,  1.0,  0.0,  0.0],
                               [  -1.0,  0.0,  0.0,  0.0],
                               [  0.0,  0.0,  1.0,  0.0],
                               [  0.0,  0.0,  0.0,  1.0]])

if __name__ == '__main__':

    command = ["rosnode", "kill", "-a"]
    subprocess.Popen(command)

    time.sleep(2)

    command = ["roslaunch", "env_startup", "benchmark_gazebo_env_startup.launch"]
    subprocess.Popen(command)

    time.sleep(2)

    # # 从键盘中读取后开始
    # input("Press Enter to start the benchmark...")

    # 初始化节点
    rospy.init_node('start_benchmark')

    # 创建 ROS 发布者
    pcd_file_pub = rospy.Publisher('/benchmark/point_cloud_file', String, queue_size=10)

    # 结果文件夹路径
    res_data = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/res_data/"

    # 修改模型的位置参数
    position = [-1.0, 0.0, 0.0]
    orientation = [0.0, 0.0, 0.0, 1.0]
    linear_velocity = [0.0, 0.0, 0.0]
    angular_velocity = [0.0, 0.0, 0.0]

    # Spawn the realsense camera
    # 设置realsense 的初始位置
    realsense_model_pose_str = "1 0 0 0 0 0 1"
    pose_values = list(map(float, realsense_model_pose_str.split()))
    model_pose = Pose()
    model_pose.position.x = pose_values[0]
    model_pose.position.y = pose_values[1]
    model_pose.position.z = pose_values[2]
    model_pose.orientation.x = pose_values[3]
    model_pose.orientation.y = pose_values[4]
    model_pose.orientation.z = pose_values[5]
    model_pose.orientation.w = pose_values[6]

    success, message = spawn_realsense(model_pose)

    # 设置模型的初始位置
    model_pose_str = "0 0 0 0 0 0 1"

    # 遍历所有模型
    for model_type in model_dir:
        # 读取模型文件
        model_file_path = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/" + model_type + "/sdf"
        # 在 model_file_path 中读取所有模型文件
        model_files = os.listdir(model_file_path)
        # 遍历所有算法
        for method in method_type:
            # 创建文件夹
            folder_name = res_data + model_type + "_" + method
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)

            current_method_code = None
            if method == "mcmf":
                current_method_code = 0
            elif method == "nbvnet":
                current_method_code = 6
            elif method == "scvp":
                current_method_code = 7

            # 修改scvp的配置文件
            with open(scvp_config_file, "r") as f:
                lines = f.readlines()
            with open(scvp_config_file, "w") as f:
                for line in lines:
                    if "method_of_IG" in line:
                        f.write("method_of_IG: " + str(current_method_code) + "\n")
                    elif "octomap_resolution" in line: # 要保证体素数量为 32*32*32
                        if current_method_code == 6:
                            f.write("octomap_resolution: 0.006\n")
                        elif current_method_code == 7:
                            f.write("octomap_resolution: 0.006\n")
                        else:
                            f.write("octomap_resolution: 0.03\n")
                    elif "num_of_max_iteration" in line:
                        f.write("num_of_max_iteration: " + str(nbv_iter) + "\n")
                    else:
                        f.write(line)

            # 遍历所有模型文件
            for model_file in model_files:
                # 创建文件夹
                model_name = model_file.split(".")[0]
                model_folder = folder_name + "/" + model_name
                if not os.path.exists(model_folder):
                    os.makedirs(model_folder)
                elif os.path.exists(model_folder + "/done.txt"):
                    continue
                else:
                    # 清空文件夹
                    os.system("rm -rf " + model_folder + "/*")

                # 修改scvp的配置文件
                with open(scvp_config_file, "r") as f:
                    lines = f.readlines()
                with open(scvp_config_file, "w") as f:
                    for line in lines:
                        if "name_of_pcd" in line:
                            f.write("name_of_pcd: " + model_type + "/pcd/" + model_name + "\n")
                        else:
                            f.write(line)

                pcd_name = model_type + "/pcd/" + model_name

                # 复制配置文件到文件夹
                command = "cp " + scvp_config_file + " " + model_folder
                os.system(command)

                # 解析模型姿态
                pose_values = list(map(float, model_pose_str.split()))
                model_pose = Pose()
                model_pose.position.x = pose_values[0]
                model_pose.position.y = pose_values[1]
                model_pose.position.z = pose_values[2]
                model_pose.orientation.x = pose_values[3]
                model_pose.orientation.y = pose_values[4]
                model_pose.orientation.z = pose_values[5]
                model_pose.orientation.w = pose_values[6]

                # 设置模型名称
                model_name = model_file.split(".")[0]
                model_ref_frame = "world"
                model_path = model_file_path + "/" + model_file
                
                success, message = spawn_model(model_path, model_name, model_pose, model_ref_frame)
        
                if success:
                    rospy.loginfo("Model spawned successfully: %s" % message)
                else:
                    rospy.logerr("Failed to spawn model: %s" % message)

                # 设置 realsense 的初始位置
                position = [-1.0, 0.0, 0.0]
                orientation = [0.0, 0.0, 0.0, 1.0]
                success, message = set_model_state("realsense", position, orientation, linear_velocity, angular_velocity)
                if success:
                    rospy.loginfo("Model state set successfully: %s" % message)
                else:
                    rospy.logerr("Failed to set model state: %s" % message)

                time.sleep(1)

                # 启动 scvp
                command = ["roslaunch", "scvp_core", "run_scvp.launch"]
                scvp_process = subprocess.Popen(command)

                # 启动 nbv_net
                command = ["python3", "/root/work_place/pb_nbv/src/scvp_core/nbv-net/run_single_test.py", pcd_name, str(nbv_iter)]
                nbv_net_porcess = subprocess.Popen(command)

                time.sleep(3)
                
                pose_topic = "/scvp_pose"
                # nbv 迭代30次
                print("nvb_iter: ", nbv_iter)
                key_time = time.time()
                key_time_last = time.time()
                for i in range(nbv_iter):
                    
                    iter_folder = model_folder + "/iter_" + str(i+1)
                    if not os.path.exists(iter_folder):
                        os.makedirs(iter_folder)

                    print("Waiting for pose data...")
                    pose = rospy.wait_for_message(pose_topic, Pose)
                    key_time = time.time()

                    nbv_matrix = pose_to_matrix(pose)

                    # 新建txt文件保存 NBV 位姿
                    pose_file = iter_folder + "/nbv_pose.txt"
                    with open(pose_file, "w") as f:
                        f.write(str(nbv_matrix))
                    
                    # 将 nbv_matrix 转换为gazebo中的相机位姿
                    nbv_matrix = nbv_matrix @ camera_diff_matrix 
                    nbv_matrix = nbv_matrix @ np.linalg.inv(base_to_depth)

                    # 新建txt文件保存时间
                    time_file = iter_folder + "/time.txt"
                    with open(time_file, "w") as f:
                        f.write(str(key_time - key_time_last))

                    # 把 nbv_matrix 转换成四元数
                    position, orientation = matrix_to_quaternion(nbv_matrix)

                    # 保存一下相机的gazebo位姿
                    pose_file = iter_folder + "/gazebo_pose.txt"
                    with open(pose_file, "w") as f:
                        f.write(str(position) + " " + str(orientation))

                    # 修改相机位姿
                    # 移动 realsense 相机
                    success, message = set_model_state("realsense", position, orientation, linear_velocity, angular_velocity)
                    if success:
                        rospy.loginfo("Model state set successfully: %s" % message)
                    else:
                        rospy.logerr("Failed to set model state: %s" % message)

                    time.sleep(1)

                    # 保存点云数据
                    # 从话题 "/d435/depth/color/points" 中获取一帧点云数据
                    print("Waiting for point cloud data...")
                    point_cloud_data = rospy.wait_for_message("/d435/depth/color/points", PointCloud2)
                    # 把当前的realsense相机四元数位姿转换成4x4矩阵
                    camera_pose = quaternion_to_matrix(position, orientation)
                    camera_pose = save_point_cloud(point_cloud_data, camera_pose, iter_folder, 0.001)
                    if camera_pose is None:
                        rospy.logerr("Failed to save point cloud data!")
                        exit(-1)
                    # 新建txt文件夹保存相机位姿 camera_pose
                    pose_file = iter_folder + "/camera_pose.txt"
                    with open(pose_file, "w") as f:
                        f.write(str(camera_pose))

                    pcd_file = iter_folder + "/point_cloud.pcd"

                    # 通过 ros 发布 pcd_file 路径的string消息
                    # 创建 String 消息
                    pcd_file_msg = String()
                    pcd_file_msg.data = pcd_file

                    # 发布消息
                    rospy.loginfo("Publishing PCD file path: %s", pcd_file)
                    pcd_file_pub.publish(pcd_file_msg)
                    key_time_last = time.time()

                # 在文件夹中创建一个文件
                with open(model_folder + "/done.txt", "w") as f:
                    f.write("done iteration !")
                    
                # 删除模型
                success, message = delete_model(model_name)
                if success:
                    rospy.loginfo("Model deleted successfully: %s" % message)
                else:
                    rospy.logerr("Failed to delete model: %s" % message)

                command = ["rosnode", "kill", "/scvp_core_node"]
                subprocess.Popen(command)

                # scvp_process.terminate()
                nbv_net_porcess.terminate()
                
                time.sleep(2)

