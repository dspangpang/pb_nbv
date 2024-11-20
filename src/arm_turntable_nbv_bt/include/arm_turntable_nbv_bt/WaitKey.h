#pragma once

#include "./utils.h"

namespace Turntable_NBV
{
    class WaitKey : public BT::SyncActionNode
    {
    public:
        WaitKey(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~WaitKey();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports

            // Add output ports
            ports_list.emplace(BT::OutputPort<std::string>("key_value"));
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "WaitKey is running " << this->name() << std::endl;

            // 获取输入参数

            // 结果信息
            int res = 0;

            // 从键盘中获取按键值
            std::string key_value = "";
            std::cin >> key_value;
            
            // 打印Log
            std::cout << "WaitKey log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "->OutputPorts " << std::endl;
            std::cout << "key_value: " << key_value << std::endl;
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
