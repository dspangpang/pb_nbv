#include "../include/moveit_terminal/TurntableTerminal.h"

#define ERROR -1

#define BUSY 1

#define SUCCESS 0

#define ANGLE2RAD 0.01745329251994329576923690768489


TurntableTerminal::TurntableTerminal(const ros::NodeHandle &nh,
                                    const std::string &group_name):turntable(group_name)
{
    this->nh_ = nh;

    service_move_gripper = 
            nh_.advertiseService("MoveTurntable", &TurntableTerminal::move_turntable_callback, 
                                this);

}

TurntableTerminal::~TurntableTerminal()
{

}


bool TurntableTerminal::move_turntable_callback(utils_msgs::MoveTurntable::Request& req, 
                                        utils_msgs::MoveTurntable::Response& res){

    int err = 0;

    auto is_moving = turntable.is_moveing();

    if(is_moving){

        res.result = BUSY;
        return true;
    }

    std::vector<double> target_joint(1);
    target_joint[0] = req.target_angle * ANGLE2RAD;

    // 排除超出范围的情况
    if(target_joint[0] > M_PI || target_joint[0] < -M_PI){
        res.result = ERROR;
        return true;
    }
    
    int result = turntable.move_joint(target_joint, true);

    res.current_pose = turntable.get_current_link_pose("turntable_support_link");

    if(!result)
    {
        res.result = SUCCESS;
        return true;
    }else{
        res.result = ERROR;
        return true;
    }
    
}






