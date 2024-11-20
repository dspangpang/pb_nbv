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

from add_model_gazebo import spawn_model, delete_model
from add_realsense_gazebo import spawn_realsense
from set_link_state import set_model_state
from utils import quaternion_to_matrix, save_point_cloud, matrix_to_quaternion, pose_to_matrix, matrix_to_pose, base_to_depth, nbv_iter

model_dir = ["hb_models", "lm_models", "stanford_models"]
method_type = ["pb"]
pb_config_file = "/root/work_place/pb_nbv/src/pb_core/config/config.json"

if __name__ == '__main__':

    command = ["rosnode", "kill", "-a"]
    subprocess.Popen(command)

    time.sleep(2)

    command = ["roslaunch", "env_startup", "benchmark_gazebo_env_startup.launch"]
    subprocess.Popen(command)

    time.sleep(2)

    # 初始化节点
    rospy.init_node('start_benchmark')

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

                # 复制配置文件
                command = "cp " + pb_config_file + " " + model_folder
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
                
                command = ["roslaunch", "pb_core", "run_pb.launch"]
                subprocess.Popen(command)

                time.sleep(3)
                
                nbv_matrix = np.eye(4)                  
                pcd_file_path = "/point_cloud.pcd"
                # nbv 迭代30次
                print("nvb_iter: ", nbv_iter)
                for i in range(nbv_iter):
                    
                    iter_folder = model_folder + "/iter_" + str(i+1)
                    if not os.path.exists(iter_folder):
                        os.makedirs(iter_folder)

                    # call nbv algorithm
                    rospy.wait_for_service('nbv_ros_node_trigger')
                    try:
                        nbv_trigger_service = rospy.ServiceProxy('nbv_ros_node_trigger', NBVTrigger)
                        nbv_request = NBVTriggerRequest()
                        nbv_request.current_camera_pose = matrix_to_pose(nbv_matrix)
                        nbv_request.pcd_file_path = pcd_file_path
                        start_time = time.time()
                        response = nbv_trigger_service(nbv_request)
                        end_time = time.time()
                        if response.result == 0:
                            rospy.loginfo("NBV algorithm executed successfully!")
                            nbv_matrix = pose_to_matrix(response.nbv_camera_pose)

                            # 新建txt文件保存 NBV 位姿
                            pose_file = iter_folder + "/nbv_pose.txt"
                            with open(pose_file, "w") as f:
                                f.write(str(nbv_matrix))

                            # 保存结果
                            print("Time cost: %f" % (end_time - start_time))

                            # 将 nbv_matrix 转换为gazebo中的相机位姿
                            gazebo_nbv_matrix = nbv_matrix @ np.linalg.inv(base_to_depth)
                            position, orientation = matrix_to_quaternion(gazebo_nbv_matrix)

                            # 保存一下相机的gazebo位姿
                            pose_file = iter_folder + "/gazebo_pose.txt"
                            with open(pose_file, "w") as f:
                                f.write(str(position) + " " + str(orientation))

                            # 新建txt文件保存时间
                            time_file = iter_folder + "/time.txt"
                            with open(time_file, "w") as f:
                                f.write(str(end_time - start_time))

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
                            camera_pose = save_point_cloud(point_cloud_data, camera_pose, iter_folder)
                            if camera_pose is None:
                                rospy.logerr("Failed to save point cloud data!")
                                exit(-1)
                            # 新建txt文件夹保存相机位姿 camera_pose
                            pose_file = iter_folder + "/camera_pose.txt"
                            with open(pose_file, "w") as f:
                                f.write(str(camera_pose))

                            pcd_file_path = iter_folder + "/point_cloud.pcd"


                        else:
                            rospy.logerr("NBV algorithm failed !")

                    except rospy.ServiceException as e:
                        rospy.logerr("Service call failed: %s" % e)
    
                # 在文件夹中创建一个文件
                with open(model_folder + "/done.txt", "w") as f:
                    f.write("done iteration !")

                # 删除模型
                success, message = delete_model(model_name)
                if success:
                    rospy.loginfo("Model deleted successfully: %s" % message)
                else:
                    rospy.logerr("Failed to delete model: %s" % message)

                command = ["rosnode", "kill", "/pb_core_node"]
                subprocess.Popen(command)
                time.sleep(2)

            
            

