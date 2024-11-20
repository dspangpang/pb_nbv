#pragma once

#include "./utils.h"
#include <utils_msgs/GetPose.h>

namespace Turntable_NBV
{
    class GetPose : public BT::SyncActionNode
    {
    public:
        GetPose(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~GetPose();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("link_name"));

            // Add output ports
            ports_list.emplace(BT::OutputPort<std::string>("output_pose"));
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "GetPose is running " << this->name() << std::endl;

            // 获取输入参数
            auto link_name = getInput<std::string>("link_name").value();
            
            // 结果信息
            int res = 0;
            

            // call Getpose ros服务
            ros::ServiceClient client_pose = root_nh_.serviceClient<utils_msgs::GetPose>("GetPose");

            utils_msgs::GetPose srv_pose;
            srv_pose.request.link_name = link_name;
            geometry_msgs::Pose current_pose;
            if(client_pose.call(srv_pose)){
                current_pose = srv_pose.response.pose;
            }
            else{
                print_warn("Failed to call service GetPose");
                res = -1;
            }
           
            std::string output_pose_str = poseToString(current_pose);
            setOutput("output_pose", output_pose_str);

            // 打印Log
            std::cout << "GetPose log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "link_name: " << link_name << std::endl;
            std::cout << "->OutputPorts " << std::endl;
            std::cout << "output_pose: " << output_pose_str << std::endl;
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
