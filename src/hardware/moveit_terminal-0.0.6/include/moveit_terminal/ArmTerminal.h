#pragma once

#include "moveit_utils/moveit_utils.h"
#include "utils_msgs/MoveArm.h"
#include "utils_msgs/GetPose.h"
#include "utils_msgs/GetJointValue.h"
#include <iostream>


class ArmTerminal
{
private:
    moveit_utils arm;
    ros::NodeHandle nh_;

    ros::ServiceServer service_move_arm;
    ros::ServiceServer service_get_pose;
    ros::ServiceServer service_get_joint_value;

    moveit_utils::CartesianParam cp;

    std::string move_line_method;

public:
    ArmTerminal(const ros::NodeHandle &nh, const std::string &group_name);

    ArmTerminal(const ros::NodeHandle &nh,
                const std::string &reference_link,
                const std::string &group_name);
                
    ~ArmTerminal();

private:
    bool move_arm_callback(utils_msgs::MoveArm::Request& req, 
                            utils_msgs::MoveArm::Response& res);

    bool get_pose_callback(utils_msgs::GetPose::Request& req, 
                            utils_msgs::GetPose::Response& res);

    bool get_joint_value_callback(utils_msgs::GetJointValue::Request& req, 
                                utils_msgs::GetJointValue::Response& res);
};


