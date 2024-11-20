#pragma once

#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PointStamped.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/limit_cartesian_speed.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <trac_ik/trac_ik.hpp>

#include <urdf/model.h>

// 3rdparty
class moveit_utils{

public:
    moveit_utils(const std::string &planning_group_name);

    ~moveit_utils();

private:
    // moveit 需要的参数
    moveit::planning_interface::MoveGroupInterface move_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const robot_state::JointModelGroup *joint_model_group;
    moveit::core::RobotModelConstPtr robot_model;
    std::string planning_group_name_;
    std::string end_effector_link;
    std::string reference_link;
    std::vector<geometry_msgs::Pose> way_points;

    int dof;

private:
    ros::NodeHandle nh_;

private:
    // 机械臂的运动链
    KDL::Chain control_chain;

private:

    int add_moveit_init_scene();

private:

    std::string current_dir;
    bool enable_trajectory_save;
    
public:

    // 笛卡尔规划变量结构体
    struct CartesianParam {
        // 成员变量声明
        double eef_step;
        double jump_threshold;
        int try_times;
        int trajectory_try_times;
        double speed_factor;
        double singularity_threshold;
        double velocity_diff_torlerance;
    }; 

public:

    // 用户函数

    /**
     * @brief 设置规划组的速度参数
     * @param vel_factor 速度参数，取值范围0~1
     * @param acc_factor 加速度参数，取值范围0~1
    */
    void set_move_group_speed(const double &vel_factor, 
                              const double &acc_factor);

    /**
     * @brief 设置规划组的允许误差
     * @param position 位置允许误差
     * @param joints 关节角度允许误差
     * @param orientation 姿态允许误差
    */
    void set_goal_tolerance(const double &position,
                        const double &joints, 
                        const double &orientation);
    
    /**
     * @brief 设置获取机械臂末端位姿的参考Link
     * @param position 位置允许误差
     * @param joints 关节角度允许误差
     * @param orientation 姿态允许误差
    */
    void set_reference_link(const std::string &reference_link);

    /**
     * @brief 通过设置关节角度移动机械臂
     * @param joint_positions_target 目标的关节角度
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_joint(const std::vector<double> &joint_positions_target, bool is_block);

    /**
     * @brief 通过设置目标位姿，对关节角度插值移动机械臂
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_pose(const geometry_msgs::Pose &target_pose, bool is_block);

    /**
     * @brief 通过设置目标位姿，笛卡尔规划运动
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_line(
        const geometry_msgs::Pose &pose,
        const CartesianParam &cp,
        bool is_block);

    /**
     * @brief 通过设置目标位姿中间插值,笛卡尔规划运动,强制执行,直到错误而退出
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_line_middle_interpolation_force(
        const geometry_msgs::Pose &pose,
        const CartesianParam &cp,
        bool is_block);

    /**
     * @brief 通过设置目标位姿中间插值，笛卡尔规划运动
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_line_middle_interpolation(
        const geometry_msgs::Pose &pose,
        const CartesianParam &cp,
        bool is_block);

    /**
     * @brief 通过设置目标位姿，笛卡尔规划运动
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_line_joint_complete(
        const geometry_msgs::Pose &pose,
        const CartesianParam &cp,
        bool is_block);

    /**
     * @brief 通过添加末端扰动，笛卡尔规划运动
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_line_end_disturbance(
        const geometry_msgs::Pose &pose,
        const CartesianParam &cp,
        bool is_block);

    /**
     * @brief 通过设置目标位姿，笛卡尔规划运动多个路径点
     * @param target_pose 目标位姿
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_waypoints (const std::vector<geometry_msgs::Pose> &waypoints,
                        const CartesianParam &cp,
                        bool is_block);

    /**
     * @brief 移动机械臂到预设位姿
     * @param prefix_pos 预设位姿的代号
     * @param is_block 是否为阻塞运动
     * @return 0:成功; -1失败
    */
    int move_to_prefix_pos(const std::string prefix_pos, bool is_block); 

    /**
     * @brief 获取当前时间
     * @return "%Y-%m-%d %H:%M:%S"
    */
    std::string getTime();

    /**
     * @brief 根据当前时间生成保存数据文件夹
     * @return 文件夹路径
    */
    std::string getTimeDir(const std::string &func);

    /**
     * @brief 把当前位姿写进文件 x y z w x y z
     * @return "%Y-%m-%d %H:%M:%S"
    */
    int write_end_pose_to_file(const std::string &path);

    /**
     * @brief 获取当前某link的位姿
     * @param link_name 要获取位姿的名称
     * @return 当前link位姿
    */
    geometry_msgs::Pose get_current_link_pose(const std::string &link_name);

    /**
     * @brief 获取当前末端的位姿
     * @return 当前末端位姿
    */
    geometry_msgs::Pose get_current_end_pose();

    /**
     * @brief 获取当前关节角度
     * @return 当前关节角度
    */
    std::vector<double> get_current_joint_state();

    /**
     * @brief 判读机械臂是否在移动
     * @return true:正在移动 false:不在移动
    */
    bool is_moveing();

private:
    
    /**
     * @brief 通过评估当前规划的关节速度推断当前路径是否经过奇异点
     * @param trajectory 待判断的规划
     * @return true:合理的规划; false:不合理的规划
    */
    bool evoluate_trajectory_velocity(const moveit_msgs::RobotTrajectory &trajectory, const CartesianParam &cp);

    /**
     * @brief 通过评估当前规划的关节速度推断当前路径是否经过奇异点
     * @param trajectory 待判断的规划
     * @param error_index 输出的错误点的索引
     * @return 反回错误点的数量
    */
    int evoluate_trajectory_velocity(
        const moveit_msgs::RobotTrajectory &trajectory,
        std::vector<int> &error_index,
        const CartesianParam &cp);

    /**
     * @brief 通过评估当前规划的雅可比矩阵推断当前路径是否经过奇异点
     * @param trajectory 待判断的规划
     * @param error_jacobian 输出的错误雅可比矩阵
     * @param error_index 输出的错误点的索引
     * @return 反回错误点的数量
    */
    int evoluate_trajectory_Jacobian(
        const moveit_msgs::RobotTrajectory &trajectory,
        std::vector<KDL::Jacobian> &error_jacobian,
        std::vector<int> &error_index,
        const CartesianParam &cp);

    /**
     * @brief 更正当前规划(二分插值),直接执行
     * @param trajectory 待更正的规划
     * @param trajectory_corrected 更正后的规划
     * @param target_pose 目标位姿
     * @param cp 笛卡尔规划结构体参数
     * @return 0:更正成功; -1:更正失败
    */
    void
    correction_trajectory_middle_interpolation_force(
        const moveit_msgs::RobotTrajectory &trajectory,
        const geometry_msgs::Pose &target_pose,
        const CartesianParam &cp,
        double * current_state,
        const double &state_step);

    /**
     * @brief 更正当前规划(二分插值)
     * @param trajectory 待更正的规划
     * @param trajectory_corrected 更正后的规划
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿
     * @param cp 笛卡尔规划结构体参数
     * @return 0:更正成功; -1:更正失败
    */
    int 
    correction_trajectory_middle_interpolation(
        const moveit_msgs::RobotTrajectory &trajectory,
        moveit_msgs::RobotTrajectory &trajectory_corrected,
        const std::vector<double> &current_joint_value,
        const geometry_msgs::Pose &target_pose,
        const CartesianParam &cp);
    
    /**
     * @brief 更正当前规划,使用关节插值进行补全
     * @param trajectory 待更正的规划
     * @param trajectory_corrected 更正后的规划
     * @return 0:更正成功; -1:更正失败
    */
    int 
    correction_trajectory_joint_complete(
        const moveit_msgs::RobotTrajectory &trajectory,
        moveit_msgs::RobotTrajectory &trajectory_corrected,
        const CartesianParam &cp);

    /**
     * @brief 更正当前规划,添加末端扰动规避奇异点
     * @param trajectory 待更正的规划
     * @param trajectory_corrected 更正后的规划
     * @return 0:更正成功; -1:更正失败
    */
    int 
    correction_trajectory_end_disturbance(
        const moveit_msgs::RobotTrajectory &trajectory,
        const geometry_msgs::Pose &target_pose,
        moveit_msgs::RobotTrajectory &trajectory_corrected,
        const CartesianParam &cp);

    /**
     * @brief 计算两个位姿之间的中间值
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿
     * @return 两个位姿之间的中间值
    */
    geometry_msgs::Pose 
    midpoint_interpolation(const geometry_msgs::Pose &current_pose,
                            const geometry_msgs::Pose &target_pose);

    /**
     * @brief 计算两个位姿之间的位置中间值,中间值的姿态是target的姿态
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿
     * @return 两个位姿之间的中间值
    */
    geometry_msgs::Pose 
    midpoint_interpolation_position(const geometry_msgs::Pose &current_pose,
                            const geometry_msgs::Pose &target_pose);

    /**
     * @brief 合并两个规划
     * @param trajectory1 规划1
     * @param trajectory2 规划2
     * @return 合并后的规划
    */
    moveit_msgs::RobotTrajectory 
    mergeTrajectories(const moveit_msgs::RobotTrajectory& trajectory1,
                        const moveit_msgs::RobotTrajectory& trajectory2);

    /**
     * @brief 使用当前关节状态计算笛卡尔路径
     * @param cp 笛卡尔规划参数
     * @param waypoints 规划覆盖的点
     * @param trajectory 计算出的规划
     * @return true: 计算成功 false: 计算失败
    */
    bool
    try_compute_cartesian_path(const CartesianParam &cp,
                                const std::vector<geometry_msgs::Pose> &waypoints,
                                moveit_msgs::RobotTrajectory &trajectory);
    /**
     * @brief 使用输入关节状态计算笛卡尔路径
     * @param cp 笛卡尔规划参数
     * @param start_joint_value 起始关节状态
     * @param waypoints 规划覆盖的点
     * @param trajectory 计算出的规划
     * @return true: 计算成功 false: 计算失败
    */
    bool 
    try_compute_cartesian_path(const CartesianParam &cp,
                                std::vector<double> start_joint_value,
                                const std::vector<geometry_msgs::Pose> &waypoints,
                                moveit_msgs::RobotTrajectory &trajectory);
    
    /**
     * @brief 判断两个位姿之间的有效距离是否小于笛卡尔规划的计算间隔
     * @param a 位姿a
     * @param b 位姿b
     * @param cp 笛卡尔规划参数
     * @return true: 有效间隔，大于计算间隔 false: 无效间隔，已经小于计算间隔无法继续插值
    */
    bool is_vaild_distance(const geometry_msgs::Pose& a,
                        const geometry_msgs::Pose& b,
                        const CartesianParam &cp);

    /**
     * @brief 计算输入关节角度的末端的雅可比矩阵
     * @param joint_values 机械臂关节角度
     * @param jacobian 计算出的雅可比矩阵
     * @return true: 计算成功 false: 计算失败
    */
    bool getEndJacobian(const std::vector<double> &joint_values,
                    KDL::Jacobian &jacobian);


    /**
     * @brief 设置机械臂的状态为当前状态
    */
    void stateReset();

    /**
     * @brief 设置机械臂的状态为输入的关节角度
     * @param joint_values 机械臂关节角度
    */
    void stateChange(const std::vector<double> &joint_values);

    /**
     * @brief 给输入位姿添加扰动
     * @param input_pose 输入位姿
     * @param position_tolerance 位置扰动范围
     * @param angle_tolerance 角度扰动范围
     * @return 输入添加末端的位姿
    */
    geometry_msgs::Pose 
    add_pose_disturbance(
        const geometry_msgs::Pose &input_pose,
        const double position_tolerance,
        const double angle_tolerance);

    /**
     * @brief 对生成后的轨迹进行时间最优重整
     * @param trajectory 待重整轨迹
    */
    void
    iptp_trajectory(moveit_msgs::RobotTrajectory& trajectory);

    /**
     * @brief 对生成后的轨迹调速
     * @param trajectory 待重整轨迹
    */
    void
    scale_trajectory_speed(moveit_msgs::RobotTrajectory& trajectory, const double &scale);
    

    /**
     * @brief 计算机械臂的正运动学
     * @param joint_values 关节角度
     * @return 输入末端的位姿
    */
    geometry_msgs::Pose
    compute_forward_kinematics(const std::vector<double>joint_values);

    /**
     * @brief 从/robot_description中获取机械臂的运动链
     * @return 输入末端的位姿
    */
    void
    init_manipulator_chain();


    /**
     * @brief 把生成的轨迹信息写入文件
     * @param path 文件夹路径
     * @param trajectory 规划路径
     * @return 成功0, 失败1
    */
    int save_trajectory_to_file(const std::string &path, const moveit_msgs::RobotTrajectory &trajectory);

};
