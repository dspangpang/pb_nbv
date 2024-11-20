#include "../include/moveit_terminal/TurntableTerminal.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "turntable_terminal_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);

    spinner.start();

    std::string group_name;
    double liner_weight;
    double liner_bias;
    double max_distance;

    const auto& node_name = ros::this_node::getName();
    
    nh.getParam(node_name + "/group_name", group_name);

    TurntableTerminal tt(nh, group_name);

    ros::Rate rate(50);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        
    }
    
}


