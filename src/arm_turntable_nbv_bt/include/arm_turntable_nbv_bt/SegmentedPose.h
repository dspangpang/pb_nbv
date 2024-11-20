#pragma once

#include "./utils.h"
#include <utils_msgs/GetPose.h>

inline geometry_msgs::Pose 
midpoint_interpolation(
    const geometry_msgs::Pose &current_pose,
    const geometry_msgs::Pose &target_pose){

    Eigen::Quaterniond current_quaternion(current_pose.orientation.w, 
                                            current_pose.orientation.x, 
                                            current_pose.orientation.y, 
                                            current_pose.orientation.z);

    Eigen::Quaterniond target_quaternion(current_pose.orientation.w, 
                                            current_pose.orientation.x, 
                                            current_pose.orientation.y, 
                                            current_pose.orientation.z);

    Eigen::Quaterniond mid_quaternion = current_quaternion.slerp(0.5, target_quaternion);

    geometry_msgs::Pose mid_pose;

    mid_pose.orientation.w = mid_quaternion.w();
    mid_pose.orientation.x = mid_quaternion.x();
    mid_pose.orientation.y = mid_quaternion.y();
    mid_pose.orientation.z = mid_quaternion.z();

    mid_pose.position.x = current_pose.position.x + 
                            (target_pose.position.x - current_pose.position.x) / 2;

    
    mid_pose.position.y = current_pose.position.y + 
                            (target_pose.position.y - current_pose.position.y) / 2; 

    
    mid_pose.position.z = current_pose.position.z + 
                            (target_pose.position.z - current_pose.position.z) / 2; 

    return mid_pose;
    
}

namespace Turntable_NBV
{
    class SegmentedPose : public BT::SyncActionNode
    {
    public:
        SegmentedPose(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~SegmentedPose();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("input_pose"));

            // Add output ports
            ports_list.emplace(BT::OutputPort<std::string>("output_pose"));
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "SegmentedPose is running " << this->name() << std::endl;

            // 获取输入参数
            auto input_pose_str = getInput<std::string>("input_pose").value();

            // 结果信息
            int res = 0;
            
            // 变量转换
            geometry_msgs::Pose input_pose = analyseTargetPose(input_pose_str);

            // call service 获取当前的末端位姿
            ros::ServiceClient client_pose = root_nh_.serviceClient<utils_msgs::GetPose>("GetPose");
            utils_msgs::GetPose srv_pose;
            srv_pose.request.link_name = "dummy_gripper";
            geometry_msgs::Pose current_end_pose;
            if(client_pose.call(srv_pose)){
                current_end_pose = srv_pose.response.pose;
            }
            else{
                print_warn("Failed to call service GetPose");
                res = -1;
            }
            
            geometry_msgs::Pose output_pose = midpoint_interpolation(current_end_pose, input_pose);

            // 设置输出参数
            std::string output_pose_str = poseToString(output_pose);
            setOutput("output_pose", output_pose_str);
            // 打印Log
            std::cout << "SegmentedPose log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->input_pose_str " << input_pose_str << std::endl;
            std::cout << "->OutputPorts " << std::endl;
            std::cout << "--->output_pose_str " << output_pose_str << std::endl;
            std::cout << "res: " << res << std::endl;  

            BT::NodeStatus status ;

            if(res == 0){
                status = BT::NodeStatus::SUCCESS;
            }

            else{
                status = BT::NodeStatus::FAILURE;
            }

            return status;
            
        }

    private:
        ros::NodeHandle root_nh_;
        ros::NodeHandle tree_nh_;
    };

} // namespace Turntable_NBV
