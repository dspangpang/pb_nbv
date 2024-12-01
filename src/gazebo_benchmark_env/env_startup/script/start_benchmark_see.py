import open3d as o3d
import numpy as np
import time
import sys
import os
import threading
import subprocess

import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from utils_msgs.srv import NBVTrigger, NBVTriggerRequest
from std_msgs.msg import String

from add_model_gazebo import spawn_model, delete_model
from add_realsense_gazebo import spawn_realsense
from set_link_state import set_model_state
from utils import quaternion_to_matrix, save_point_cloud, matrix_to_quaternion, matrix_to_pose, gazebo_pose_to_tf, parse_pointcloud2, read_rho_from_launch, modify_tho_launch_file, base_to_depth, nbv_iter

model_dir = ["hb_models", "lm_models", "stanford_models"]
# model_dir = ["hb_models"]
method_type = ["see"]
see_config_file = "/root/work_place/pb_nbv/src/see_core/launch/run_see.launch"
default_rho = 1500000
# 最小迭代次数
min_iter = 10

# 初始化realsense pose 的 tf 变换
realsense_pose = Pose()
realsense_pose.position.x = 0.0

realsense_pose.position.y = 0.0
realsense_pose.position.z = 0.0
realsense_pose.orientation.x = 0.0
realsense_pose.orientation.y = 0.0
realsense_pose.orientation.z = 0.0
realsense_pose.orientation.w = 1.0

def pub_fake_gazebo_tf():
    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(30) 

    while not rospy.is_shutdown():
        dynamic_transformStamped = geometry_msgs.msg.TransformStamped()

        dynamic_transformStamped.header.stamp = rospy.Time.now()
        dynamic_transformStamped.header.frame_id = "world"
        dynamic_transformStamped.child_frame_id = "d435_depth_optical_frame"

        dynamic_transformStamped.transform.translation.x = realsense_pose.position.x
        dynamic_transformStamped.transform.translation.y = realsense_pose.position.y
        dynamic_transformStamped.transform.translation.z = realsense_pose.position.z
        dynamic_transformStamped.transform.rotation.x = realsense_pose.orientation.x
        dynamic_transformStamped.transform.rotation.y = realsense_pose.orientation.y
        dynamic_transformStamped.transform.rotation.z = realsense_pose.orientation.z
        dynamic_transformStamped.transform.rotation.w = realsense_pose.orientation.w

        broadcaster.sendTransform(dynamic_transformStamped)

        rate.sleep()

if __name__ == '__main__':

    command = ["rosnode", "kill", "-a"]
    subprocess.Popen(command)

    time.sleep(2)

    command = ["roslaunch", "env_startup", "benchmark_gazebo_env_startup.launch"]
    subprocess.Popen(command)

    time.sleep(2)

    # 初始化节点
    rospy.init_node('start_benchmark')

    #新建一个线程发布静态的 TF 变换
    t = threading.Thread(target=pub_fake_gazebo_tf)
    t.start()

    print("Start benchmarking...")
    
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

    # # 从键盘中读取后开始
    # input("Press Enter to start the benchmark...")

    # 遍历所有模型
    for model_type in model_dir:
        # 读取模型文件
        model_file_path = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/" + model_type + "/sdf"
        # 在 model_file_path 中读取所有模型文件
        model_files = os.listdir(model_file_path)
        print("Current model type: ", model_type)
        # 遍历所有算法
        for method in method_type:
            # 创建文件夹
            folder_name = res_data + model_type + "_" + method
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
            # 遍历所有模型文件
            for model_file in model_files:
                # 创建文件夹
                current_rho = read_rho_from_launch(see_config_file)
                model_name = model_file.split(".")[0]
                model_folder = folder_name + "/" + model_name
                if not os.path.exists(model_folder):
                    os.makedirs(model_folder)
                    modify_tho_launch_file(see_config_file, default_rho)
                elif os.path.exists(model_folder + "/done.txt"):
                    continue
                else:
                    # 清空文件夹
                    os.system("rm -rf " + model_folder + "/*")

                # 复制配置文件
                command = "cp " + see_config_file + " " + model_folder
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
                position = [0.9, 1.75000000e-02, -1.25000000e-02]
                orientation = [0.0, 0.0, 1.0, 0.0]
                tmp_position = base_to_depth

                success, message = set_model_state("realsense", position, orientation, linear_velocity, angular_velocity)
                if success:
                    rospy.loginfo("Model state set successfully: %s" % message)
                    tf_pose = gazebo_pose_to_tf(position, orientation)
                    with threading.Lock():
                        realsense_pose.position.x = tf_pose.position.x
                        realsense_pose.position.y = tf_pose.position.y
                        realsense_pose.position.z = tf_pose.position.z
                        realsense_pose.orientation.x = tf_pose.orientation.x
                        realsense_pose.orientation.y = tf_pose.orientation.y
                        realsense_pose.orientation.z = tf_pose.orientation.z
                        realsense_pose.orientation.w = tf_pose.orientation.w
                else:
                    rospy.logerr("Failed to set model state: %s" % message)

                time.sleep(1)

                # 创建第一帧文件夹
                iter_folder = model_folder + "/iter_1"
                if not os.path.exists(iter_folder):
                    os.makedirs(iter_folder)

                # 在第一帧文件夹中保存一帧点云数据
                point_cloud_data = rospy.wait_for_message("/d435/depth/color/points", PointCloud2)
                camera_pose = quaternion_to_matrix(position, orientation)
                camera_pose = save_point_cloud(point_cloud_data, camera_pose, iter_folder)
                if camera_pose is None:
                    rospy.logerr("Failed to save point cloud data!")
                    exit(-1)
                # 新建txt文件夹保存相机位姿 camera_pose
                pose_file = iter_folder + "/camera_pose.txt"
                with open(pose_file, "w") as f:
                    f.write(str(camera_pose))

                # 新建文件夹保存 耗时
                time_file = iter_folder + "/time.txt"
                with open(time_file, "w") as f:
                    f.write("0")

                # 新建文件夹保存 NBV 位姿
                pose_file = iter_folder + "/nbv_pose.txt"
                with open(pose_file, "w") as f:
                    f.write(str(camera_pose))


                command = ["roslaunch", "see_core", "run_see.launch"]
                subprocess.Popen(command)

                time.sleep(3)

                start_trig = False
                param = "/see/benchmark_start"
                if rospy.has_param(param):
                    start_trig = rospy.delete_param(param)

                rospy.set_param(param, False)
                if rospy.has_param(param):
                    start_trig = rospy.get_param(param)
                else:
                    print("No param named %s" % param)
                    exit(-1)

                # nbv 迭代30次
                nbv_done = False
                print("nvb_iter: ", nbv_iter)
                current_iter = 1
                for i in range(1, nbv_iter):
                    iter_folder = model_folder + "/iter_" + str(i+1)
                    if not os.path.exists(iter_folder):
                        os.makedirs(iter_folder)
                    
                    # 触发 NBV 算法
                    rospy.set_param(param, True)
                    start_time = time.time()

                    # 等待 nbv 结果
                    start_trig = rospy.get_param(param)
                    while start_trig:
                        nbv_done = rospy.get_param("/see/done")
                        start_trig = rospy.get_param(param)
                        rospy.sleep(0.01)  # 添加一个小的延迟，避免过度占用 CPU
                    if nbv_done:
                        # 在文件夹中创建一个文件
                        with open(model_folder + "/done.txt", "w") as f:
                            f.write("done successfully!")
                        # 跳出循环
                        break

                    see_nbv_msg = rospy.wait_for_message("/see/see_nbv", PointCloud2)
                    end_time = time.time()
                    
                    # 新建txt文件保存时间
                    time_file = iter_folder + "/time.txt"
                    with open(time_file, "w") as f:
                        f.write(str(end_time - start_time))

                    nbv_matrix = parse_pointcloud2(see_nbv_msg)

                    # 新建txt文件保存 NBV 位姿
                    pose_file = iter_folder + "/nbv_pose.txt"
                    with open(pose_file, "w") as f:
                        f.write(str(nbv_matrix))

                    # 将 nbv_matrix 转换 geometry_msgs/Pose
                    nbv_pose = matrix_to_pose(nbv_matrix)

                    with threading.Lock():
                        realsense_pose.position.x = nbv_pose.position.x
                        realsense_pose.position.y = nbv_pose.position.y
                        realsense_pose.position.z = nbv_pose.position.z
                        realsense_pose.orientation.x = nbv_pose.orientation.x
                        realsense_pose.orientation.y = nbv_pose.orientation.y
                        realsense_pose.orientation.z = nbv_pose.orientation.z
                        realsense_pose.orientation.w = nbv_pose.orientation.w

                    # 将 nbv_matrix 转换为gazebo中的相机位姿
                    nbv_matrix = nbv_matrix @ np.linalg.inv(base_to_depth)

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

                    current_iter += 1

                if current_iter < min_iter:
                    # 在文件夹中创建一个文件
                    with open(model_folder + "/less_iter.txt", "w") as f:
                        f.write("done iteration !")
                else:
                    # 如果迭代次数大于等于 min_iter，则删除 less_iter.txt 文件
                    if os.path.exists(model_folder + "/less_iter.txt"):
                        os.remove(model_folder + "/less_iter.txt")
                        
                # 在文件夹中创建一个文件
                with open(model_folder + "/done.txt", "w") as f:
                    f.write("done iteration !")

                # 删除模型
                success, message = delete_model(model_name)
                if success:
                    rospy.loginfo("Model deleted successfully: %s" % message)
                else:
                    rospy.logerr("Failed to delete model: %s" % message)

                command = ["rosnode", "kill", "/see"]
                subprocess.Popen(command)
                time.sleep(2)

                modify_tho_launch_file(see_config_file, current_rho)

            
            


            
            

