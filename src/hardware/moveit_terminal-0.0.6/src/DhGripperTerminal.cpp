#include "../include/moveit_terminal/DhGripperTerminal.h"

#define ERROR -1

#define BUSY 1

#define SUCCESS 0


DhGripperTerminal::DhGripperTerminal(const ros::NodeHandle &nh,
                                    const std::string &group_name,
                                    const double max_distance,
                                    const double liner_weight,
                                    const double liner_bias):gripper(group_name)
{
    this->nh_ = nh;

    service_move_gripper = 
            nh_.advertiseService("MoveDhGripper", &DhGripperTerminal::move_gripper_callback, 
                                this);

    this->liner_weight_ = liner_weight;
    this->liner_bias_ = liner_bias;

    this->max_distance_ = max_distance;
}

DhGripperTerminal::~DhGripperTerminal()
{
}


bool DhGripperTerminal::move_gripper_callback(utils_msgs::MoveDhGripper::Request& req, 
                                        utils_msgs::MoveDhGripper::Response& res){

    int err = 0;

    auto is_moving = gripper.is_moveing();

    if(is_moving){

        res.result = BUSY;
        return true;
    }

    double target_distance;

    std::vector<double> target_joint(1);

    // 排除超出范围的情况
    if(req.target_distance > max_distance_ ){
        target_distance = max_distance_;
    }
    else if(req.target_distance < 0){
        target_distance = 0;
    }
    else{
        target_distance = req.target_distance;
    }

    // 从张开距离换算到夹爪关节值
    target_joint[0] = target_distance * liner_weight_ + liner_bias_;

    std::cout << "target_distance: " << target_distance << std::endl;
    std::cout << "target_joint: " << target_joint[0] << std::endl;
    
    int result = gripper.move_joint(target_joint, true);

    if(!result)
    {
        res.result = SUCCESS;
        return true;
    }else{
        res.result = ERROR;
        return true;
    }
    
}






