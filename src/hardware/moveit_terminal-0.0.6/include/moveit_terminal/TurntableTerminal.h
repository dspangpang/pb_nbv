#pragma once

#include "moveit_utils/moveit_utils.h"
#include "utils_msgs/MoveTurntable.h"
#include <iostream>


class TurntableTerminal
{
private:
    moveit_utils turntable;
    ros::NodeHandle nh_;

    ros::ServiceServer service_move_gripper;

public:
    TurntableTerminal(const ros::NodeHandle &nh,
                        const std::string &group_name);
    ~TurntableTerminal();

private:
    bool move_turntable_callback(utils_msgs::MoveTurntable::Request& req, 
                                utils_msgs::MoveTurntable::Response& res);
};


