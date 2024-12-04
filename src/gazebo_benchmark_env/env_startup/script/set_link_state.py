#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist

def set_model_state(model_name, position, orientation, linear_velocity, angular_velocity):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = position[0]
        model_state.pose.position.y = position[1]
        model_state.pose.position.z = position[2]
        model_state.pose.orientation.x = orientation[0]
        model_state.pose.orientation.y = orientation[1]
        model_state.pose.orientation.z = orientation[2]
        model_state.pose.orientation.w = orientation[3]
        model_state.twist.linear.x = linear_velocity[0]
        model_state.twist.linear.y = linear_velocity[1]
        model_state.twist.linear.z = linear_velocity[2]
        model_state.twist.angular.x = angular_velocity[0]
        model_state.twist.angular.y = angular_velocity[1]
        model_state.twist.angular.z = angular_velocity[2]
        model_state.reference_frame = "world"
        response = set_model_state_service(model_state)
        return response.success, response.status_message
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, str(e)

if __name__ == "__main__":
    rospy.init_node('set_model_state_example')
    model_name = "d435"
    position = [1.0, 2.0, 3.0]
    orientation = [0.0, 0.0, 0.0, 1.0]
    linear_velocity = [0.0, 0.0, 0.0]
    angular_velocity = [0.0, 0.0, 0.0]
    success, message = set_model_state(model_name, position, orientation, linear_velocity, angular_velocity)
    if success:
        rospy.loginfo("Model state set successfully: %s" % message)
    else:
        rospy.logerr("Failed to set model state: %s" % message)