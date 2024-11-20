#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <sensor_msgs/JointState.h>

#include <thread>

#include <iostream>
#include <unistd.h>
#define imsleep(microsecond) usleep(1000 * microsecond) // ms
#include <vector>

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"



class TurntableRosControl {
public:
    explicit TurntableRosControl(const ros::NodeHandle &nh, const std::string &action_name);

    ~TurntableRosControl();

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    std::string action_name_;
    std::vector<std::string> joint_names_;

    // create messages that are used to published feedback/result
    control_msgs::FollowJointTrajectoryFeedback feedback_;
    control_msgs::FollowJointTrajectoryResult result_;
    
private:
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

};


