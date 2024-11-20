#include "../include/moveit_terminal/ArmTerminal.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "arm_terminal_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);

    spinner.start();

    std::string group_name;
    std::string reference_link;

    const auto& node_name = ros::this_node::getName();
    nh.getParam(node_name + "/group_name", group_name);
    nh.getParam(node_name + "/reference_link", reference_link);


    ArmTerminal at(nh, reference_link, group_name);


    ros::Rate rate(50);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        
    }
    
}


