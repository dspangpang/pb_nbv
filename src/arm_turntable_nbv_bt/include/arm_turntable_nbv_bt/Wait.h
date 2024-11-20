#pragma once

#include "./utils.h"

namespace Turntable_NBV
{
    class Wait : public BT::SyncActionNode
    {
    public:
        Wait(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~Wait();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("sleep_time_ms"));

            // Add output ports
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "Wait is running " << this->name() << std::endl;

            // 获取输入参数
            auto sleep_time_ms_str = getInput<std::string>("sleep_time_ms").value();
            double sleep_time_ms = std::stod(sleep_time_ms_str);

            // 结果信息
            int res = 0;

            // 从键盘中获取按键值
            usleep(sleep_time_ms * 1000.0);
            
            // 打印Log
            std::cout << "Wait log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->sleep_time_ms " << sleep_time_ms << std::endl;
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
