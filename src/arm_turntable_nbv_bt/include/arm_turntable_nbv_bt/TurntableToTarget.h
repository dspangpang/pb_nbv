#pragma once

#include "./utils.h"
#include <utils_msgs/MoveTurntable.h>

namespace Turntable_NBV
{
    class TurntableToTarget : public BT::SyncActionNode
    {
    public:
        TurntableToTarget(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~TurntableToTarget();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("target_angle"));
            ports_list.emplace(BT::OutputPort<std::string>("current_pose"));

            // add output ports
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "TurntableToTarget is running " << this->name() << std::endl;

            // 获取输入参数
            auto turntable_angle_str = getInput<std::string>("target_angle").value();

            // 结果信息
            int res = 0;

            // 变量转换
            double turntable_angle = std::stod(turntable_angle_str);

            // call MoveTurntable ros服务
            ros::ServiceClient client = root_nh_.serviceClient<utils_msgs::MoveTurntable>("MoveTurntable");
            utils_msgs::MoveTurntable srv;
            srv.request.target_angle = turntable_angle;
            geometry_msgs::Pose current_pose;
            if(client.call(srv)){
                
                current_pose = srv.response.current_pose;
                res = srv.response.result;
            }
            else{
                print_warn("Failed to call service MoveTurntable");
                res = -1;
            }

            // 输出参数
            setOutput("current_pose", poseToString(current_pose));

            //打印信息
            std::cout << "TurntableToTarget log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->turntable_angle " << turntable_angle << std::endl;
            std::cout << "->OutputPorts" << std::endl;
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
