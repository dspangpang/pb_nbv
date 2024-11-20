#include "../include/moveit_terminal/ArmTerminal.h"

#define ERROR -1

#define BUSY 1

#define SUCCESS 0


ArmTerminal::ArmTerminal(const ros::NodeHandle &nh,
                            const std::string &group_name):arm(group_name)
{
    this->nh_ = nh;

    service_move_arm = 
            nh_.advertiseService("MoveArm", &ArmTerminal::move_arm_callback, this);

    ROS_INFO("MoveArm service is ready");

    service_get_pose = 
            nh_.advertiseService("GetPose", &ArmTerminal::get_pose_callback, this);

    ROS_INFO("GetPose service is ready");
    
    service_get_joint_value = 
            nh_.advertiseService("GetJointValue", &ArmTerminal::get_joint_value_callback, this);

    ROS_INFO("GetJointValue service is ready");

    const auto& node_name = ros::this_node::getName();

    nh.getParam(node_name + "/eef_step", cp.eef_step);
    nh.getParam(node_name + "/jump_threshold", cp.jump_threshold);
    nh.getParam(node_name + "/jump_threshold", cp.trajectory_try_times);
    nh.getParam(node_name + "/try_times", cp.try_times);
    nh.getParam(node_name + "/speed_factor", cp.speed_factor);
    nh.getParam(node_name + "/singularity_threshold", cp.singularity_threshold);
    nh.getParam(node_name + "/velocity_diff_torlerance", cp.velocity_diff_torlerance);

    nh.getParam(node_name + "/move_line_method", move_line_method);
}

ArmTerminal::ArmTerminal(const ros::NodeHandle &nh,
                        const std::string &reference_link,
                        const std::string &group_name):arm(group_name)
{
    this->nh_ = nh;

    service_move_arm = 
            nh_.advertiseService("MoveArm", &ArmTerminal::move_arm_callback, this);

    ROS_INFO("MoveArm service is ready");

    service_get_pose = 
            nh_.advertiseService("GetPose", &ArmTerminal::get_pose_callback, this);  

    ROS_INFO("GetPose service is ready");
    
    service_get_joint_value = 
            nh_.advertiseService("GetJointValue", &ArmTerminal::get_joint_value_callback, this);

    ROS_INFO("GetJointValue service is ready");
            
    arm.set_reference_link(reference_link);

    const auto& node_name = ros::this_node::getName();

    nh.getParam(node_name + "/eef_step", cp.eef_step);
    nh.getParam(node_name + "/jump_threshold", cp.jump_threshold);
    nh.getParam(node_name + "/try_times", cp.try_times);
    nh.getParam(node_name + "/speed_factor", cp.speed_factor);
    nh.getParam(node_name + "/singularity_threshold", cp.singularity_threshold);
    nh.getParam(node_name + "/velocity_diff_torlerance", cp.velocity_diff_torlerance);

    nh.getParam(node_name + "/move_line_method", move_line_method);


}

ArmTerminal::~ArmTerminal()
{
}


bool ArmTerminal::move_arm_callback(utils_msgs::MoveArm::Request& req, 
                                    utils_msgs::MoveArm::Response& res){

    int err = 0;

    auto is_moving = arm.is_moveing();

    if(is_moving){

        res.result = BUSY;
        return true;
    }

    switch (req.control_mode)
    {
    case 0:
        err = arm.move_pose(req.pose, true);
        
        if(err < 0){
            res.result = ERROR;
            ROS_ERROR("MovePose failed");
        }else{
            res.result = SUCCESS;
        }

        break;
    
    case 1:
        
        if (move_line_method == "move_line")
        {
            err = arm.move_line(req.pose, cp, true);
        
            if(err < 0){
                res.result = ERROR;
                ROS_ERROR("MoveLine failed");
            }else{  
                res.result = SUCCESS;
            }

            break;
        }

        if (move_line_method == "move_line_middle_interpolation")
        {
            err = arm.move_line_middle_interpolation_force(req.pose, cp, true);
        
            if(err < 0){
                res.result = ERROR;
                ROS_ERROR("MoveLine failed");
            }else{  
                res.result = SUCCESS;
            }

            break;
        }

        
        if (move_line_method == "move_line_joint_complete")
        {
            err = arm.move_line_joint_complete(req.pose, cp, true);
        
            if(err < 0){
                res.result = ERROR;
                ROS_ERROR("MoveLine failed");
            }else{  
                res.result = SUCCESS;
            }

            break;
        } 

        if (move_line_method == "move_line_end_disturbance")
        {
            err = arm.move_line_end_disturbance(req.pose, cp, true);
        
            if(err < 0){
                res.result = ERROR;
                ROS_ERROR("MoveLine failed");
            }else{  
                res.result = SUCCESS;
            }

            break;
        }         
    
    case 2:

        err = arm.move_joint(req.joint_value, true);
        
        if(err < 0){
            res.result = ERROR;
            ROS_ERROR("MoveJoint failed");
        }else{
            res.result = SUCCESS;
        }

        break;
    
    default:
        break;
    }
    
    return true;
}

bool ArmTerminal::get_pose_callback(utils_msgs::GetPose::Request& req, 
                                    utils_msgs::GetPose::Response& res){

    res.pose = arm.get_current_link_pose(req.link_name);

    res.result = SUCCESS;
    
    return true;
}

bool ArmTerminal::get_joint_value_callback(utils_msgs::GetJointValue::Request& req, 
                                        utils_msgs::GetJointValue::Response& res){

    res.joint_value = arm.get_current_joint_state();

    res.result = SUCCESS;
    
    return true;
}
