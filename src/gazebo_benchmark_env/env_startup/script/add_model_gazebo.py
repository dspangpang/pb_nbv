#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import sys

def spawn_model(model_file_path, model_name, model_pose, model_ref_frame):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # 读取模型文件
        with open(model_file_path, 'r') as model_file:
            model_xml = model_file.read()

        # 设置请求参数
        srv_request = SpawnModelRequest()
        srv_request.model_name = model_name
        srv_request.model_xml = model_xml
        srv_request.reference_frame = model_ref_frame
        srv_request.initial_pose = model_pose

        # 调用服务
        response = spawn_model_service(srv_request)
        return response.success, response.status_message
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, str(e)

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        # 设置请求参数
        srv_request = DeleteModelRequest()
        srv_request.model_name = model_name

        # 调用服务
        response = delete_model_service(srv_request)
        return response.success, response.status_message
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, str(e)

if __name__ == "__main__":
    rospy.init_node('spawn_model_example')

    # 获取输入参数
    model_name = sys.argv[1] if len(sys.argv) > 1 else "default_model"
    model_pose_str = sys.argv[2] if len(sys.argv) > 2 else "0 0 0 0 0 0 1"
    model_ref_frame = sys.argv[3] if len(sys.argv) > 3 else "world"
    
    model_type = "stanford_models"
    model_cnt = "1"
    model_file_path =f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/{model_type}/sdf/obj_00000{model_cnt}.sdf"

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

    success, message = spawn_model(model_file_path, model_name, model_pose, model_ref_frame)
    if success:
        rospy.loginfo("Model spawned successfully: %s" % message)
    else:
        rospy.logerr("Failed to spawn model: %s" % message)