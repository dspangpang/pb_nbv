#include "../include/moveit_terminal/DhGripperTerminal.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "dh_gripper_terminal_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);

    spinner.start();

    std::string group_name;
    double liner_weight;
    double liner_bias;
    double max_distance;

    const auto& node_name = ros::this_node::getName();
    
    nh.getParam(node_name + "/group_name", group_name);
    nh.getParam(node_name + "/liner_weight", liner_weight);
    nh.getParam(node_name + "/liner_bias", liner_bias);
    nh.getParam(node_name + "/max_distance", max_distance);

    DhGripperTerminal dht(nh, group_name, max_distance,liner_weight, liner_bias);

    ros::Rate rate(50);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        
    }
    
}


