#include "../include/turntable_ros_controller/TurntableRosControl.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "turntable_ros_controller_node");
    ros::NodeHandle nh;

    TurntableRosControl drc(nh, "turntable_controller/follow_joint_trajectory");

    return 0;
}

