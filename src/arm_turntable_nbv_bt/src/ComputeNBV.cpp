#include "../include/arm_turntable_nbv_bt/ComputeNBV.h"

#define SUCCESS 0
#define FAILED -1
#define BUSY 1

Turntable_NBV::ComputeNBV::ComputeNBV(const std::string& name, 
                        const BT::NodeConfiguration& config, 
                        const ros::NodeHandle& root_nh,
                        const ros::NodeHandle& tree_nh) : BT::SyncActionNode(name, config){

    this->root_nh_ = root_nh; 
    this->tree_nh_ = tree_nh;
}

Turntable_NBV::ComputeNBV::~ComputeNBV(){

}
