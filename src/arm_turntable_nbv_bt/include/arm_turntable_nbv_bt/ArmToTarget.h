#pragma once

#include "./utils.h"
#include "utils_msgs/MoveArm.h"

namespace Turntable_NBV
{
    class ArmToTarget : public BT::SyncActionNode
    {
    public:
        ArmToTarget(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~ArmToTarget();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("arm_control_mode"));
            ports_list.emplace(BT::InputPort<std::string>("joint_value"));
            ports_list.emplace(BT::InputPort<std::string>("target_pose"));

            // Add output ports
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "ArmToTarget is running " << this->name() << std::endl;

            // 获取输入参数
            auto arm_control_mode_str = getInput<std::string>("arm_control_mode").value();
            
            auto joint_value_str = getInput<std::string>("joint_value").value();

            auto target_pose_str = getInput<std::string>("target_pose").value();

            // 结果信息
            int res = 0;
            
            std::vector<double> joint_value = analyseJointValue(joint_value_str);
            ros::ServiceClient client = root_nh_.serviceClient<utils_msgs::MoveArm>("MoveArm");
            utils_msgs::MoveArm srv;

            // 变量转换
            int arm_control_mode = std::stoi(arm_control_mode_str);

            // call service
            if(arm_control_mode == 2){
                // 角度转弧度
                for(int i = 0; i < joint_value.size(); i++){
                    joint_value[i] = joint_value[i] * ANGLE2RADIAN;
                }
            }
            if(arm_control_mode == 3){
            // 3也是根据关节角度移动，以弧度的方式
                arm_control_mode = 2;
            }
            
            srv.request.joint_value = joint_value;
            srv.request.pose = analyseTargetPose(target_pose_str);
            srv.request.control_mode = arm_control_mode;
            if(client.call(srv)){
                std::cout << "MoveArm success" << std::endl;

                res = srv.response.result;
            }
            else{
                print_warn("Failed to call service MoveArm");
                res = -1;
            } 

            // 打印Log
            std::cout << "ArmToTarget log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->arm_control_mode " << arm_control_mode << std::endl;
            std::cout << "--->joint_value_str " << joint_value_str << std::endl;
            std::cout << "--->target_pose_str " << target_pose_str << std::endl;
            std::cout << "->OutputPorts " << std::endl;
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
