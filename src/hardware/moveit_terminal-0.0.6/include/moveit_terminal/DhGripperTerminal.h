#pragma once

#include "moveit_utils/moveit_utils.h"
#include "utils_msgs/MoveDhGripper.h"
#include <iostream>


class DhGripperTerminal
{
private:
    moveit_utils gripper;
    ros::NodeHandle nh_;

    ros::ServiceServer service_move_gripper;

    double liner_weight_;
    double liner_bias_;
    double max_distance_;

public:
    DhGripperTerminal(const ros::NodeHandle &nh,
                        const std::string &group_name,
                        const double max_distance,
                        const double liner_weight,
                        const double liner_bias);
    ~DhGripperTerminal();

private:
    bool move_gripper_callback(utils_msgs::MoveDhGripper::Request& req, 
                                utils_msgs::MoveDhGripper::Response& res);
};


