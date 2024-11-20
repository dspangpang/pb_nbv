#pragma once

#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

#include <sstream>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

// ros服务
#include "utils_msgs/GetPose.h"
#include "utils_msgs/GetJointValue.h"

// jsonparser
#include "./jsonparser.hpp"

namespace Turntable_NBV
{   
    #define ANGLE2RADIAN (M_PI / 180.0)
    #define RADIAN2ANGLE (180.0 / M_PI)
    /**
     * @brief 解析字符串，获取目标位姿
     * @param target_pose 目标位姿字符串
     * @return 目标位姿
    */
    geometry_msgs::Pose analyseTargetPose(const std::string &target_pose);

    /**
     * @brief 解析字符串，获取关节角
     * @param joint_value 关节角字符串
     * @return 关节角
    */
    std::vector<double> analyseJointValue(const std::string &joint_value);
    /**
     * @brief 将关节角转换为字符串
    */
    std::string jointValueTostring(const std::vector<double> &joint_value);
    /**
     * @brief 解析字符串，获取数据数组
     * @param data_arry 数据数组字符串
     * @return 数据数组
    */
    std::vector<double> analyseDataArry(const std::string &data_arry);

    /**
     * @brief 将geometry_msgs::Pose转换为Eigen::Matrix4d
     * @param pose 输入的geometry_msgs::Pose
    */
    Eigen::Matrix4d poseToMatrix(const geometry_msgs::Pose& pose);

    /**
     * @brief 将Eigen::Matrix4d转换为geometry_msgs::Pose
     * @param matrix 输入的Eigen::Matrix4d
    */
    geometry_msgs::Pose matrixToPose(const Eigen::Matrix4d& matrix);

    /**
     * @brief 将geometry_msgs::Pose转换为字符串
     * @param pose 输入的geometry_msgs::Pose
    */
    std::string poseToString(const geometry_msgs::Pose& pose);

    /**
     * @brief 获取当前末端位姿到目标位姿的角度
    */
    double get_angel(const geometry_msgs::Pose current_pos, const geometry_msgs::Pose target_pos);

    /**
     * @brief red error msg
     * @param err_msg 错误信息
    */
    void print_error(const std::string& err_msg);
    /**
     * @brief yellow warning msg
     * @param warn_msg 警告信息
    */
    void print_warn(const std::string& warn_msg);
    /**
     * @brief green success msg
     * @param success_msg 成功信息
    */
    void print_success(const std::string& success_msg);

    // ros service 相关
    bool callGetPose(ros::NodeHandle &nh,const std::string &link_name,geometry_msgs::Pose &pose);

    bool callGetJointValue(ros::NodeHandle &nh, std::vector<double> &joint_value);
    
} // namespace name


