#pragma once

#include "./utils.h"

namespace Turntable_NBV
{
    class CheckTerminatedCondition : public BT::ConditionNode
    {
    public:
        CheckTerminatedCondition(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~CheckTerminatedCondition();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("is_terminated"));

            // Add output 
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "CheckTerminatedCondition is running " << this->name() << std::endl;

            // 获取输入参数
            auto is_terminated = getInput<std::string>("is_terminated").value();

            // 结果信息
            int res = 0;

            if (is_terminated == "0")
            {
                res = 0;
            }
            else
            {
                res = -1;
            }

            //打印信息
            std::cout << "CheckTerminatedCondition log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->is_terminated " << is_terminated << std::endl;
            std::cout << "->OutputPorts" << std::endl;
            std::cout << "res: " << res << std::endl;

            BT::NodeStatus status;

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
