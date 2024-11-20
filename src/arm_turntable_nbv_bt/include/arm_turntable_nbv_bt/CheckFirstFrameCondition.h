#pragma once

#include "./utils.h"

namespace Turntable_NBV
{
    class CheckFirstFrameCondition : public BT::ConditionNode
    {
    public:
        CheckFirstFrameCondition(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~CheckFirstFrameCondition();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("nbv_cnt"));

            // Add output 
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "CheckFirstFrameCondition is running " << this->name() << std::endl;

            // 获取输入参数
            auto nbv_cnt = getInput<std::string>("nbv_cnt").value();

            // 结果信息
            int res = 0;

            if (nbv_cnt == "1")
            {
                res = 0;
            }
            else
            {
                res = -1;
            }

            //打印信息
            std::cout << "CheckFirstFrameCondition log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->nbv_cnt " << nbv_cnt << std::endl;
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
