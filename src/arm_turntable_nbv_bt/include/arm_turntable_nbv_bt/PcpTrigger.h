#pragma once

#include "./utils.h"
#include "utils_msgs/PcpTrigger.h"

namespace Turntable_NBV
{
    class PcpTrigger : public BT::SyncActionNode
    {
    public:
        PcpTrigger(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~PcpTrigger();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("input_dir_path"));

            // Add output ports
            ports_list.emplace(BT::OutputPort<std::string>("output_pcd_path"));
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "PcpTrigger is running " << this->name() << std::endl;

            // 获取输入参数
            auto input_dir_path = getInput<std::string>("input_dir_path").value();

            // 设置输出参数
            
            // 结果信息
            int res = 0;

            // 变量转换

            // 点云预处理 提取 ROI 点云 call 服务的方式
            ros::ServiceClient client = root_nh_.serviceClient<utils_msgs::PcpTrigger>("pointcloud_preprocess");
            utils_msgs::PcpTrigger srv;
            
            srv.request.input_dir_path = input_dir_path;
            std::string output_pcd_path;
            if(client.call(srv)){
                std::cout << "pointcloud_preprocess success" << std::endl;
                output_pcd_path = srv.response.output_pcd_path;
            }
            else{
                print_warn("Failed to call service pointcloud_preprocess");
                res = -1;
            }

            // 设置输出参数
            setOutput("output_pcd_path", output_pcd_path);
            
            // 打印Log
            std::cout << "PcpTrigger log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->input_dir_path " << input_dir_path << std::endl;
            std::cout << "->OutputPorts " << std::endl;
            std::cout << "--->output_pcd_path " << output_pcd_path << std::endl;
            std::cout << "--->res " << res << std::endl;
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
