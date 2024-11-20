#pragma once

#include "./utils.h"
#include "utils_msgs/gwCameraTrigger.h"

namespace Turntable_NBV
{
    class gwCameraTrigger : public BT::SyncActionNode
    {
    public:
        gwCameraTrigger(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~gwCameraTrigger();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("exposure_time"));
            ports_list.emplace(BT::InputPort<std::string>("target_data_dir"));
            ports_list.emplace(BT::InputPort<std::string>("cnt"));

            // Add output ports
            ports_list.emplace(BT::OutputPort<std::string>("current_gray_file_path"));
            ports_list.emplace(BT::OutputPort<std::string>("current_pcd_file_path"));
            ports_list.emplace(BT::OutputPort<std::string>("current_rgb_file_path"));
            ports_list.emplace(BT::OutputPort<std::string>("current_dir"));

            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "gwCameraTrigger is running " << this->name() << std::endl;

            // 获取输入参数
            auto exposure_time_str = getInput<std::string>("exposure_time").value();
            auto target_data_dir = getInput<std::string>("target_data_dir").value();
            auto cnt = getInput<std::string>("cnt").value();

            // 结果信息
            int res = 0;

            // 变量转换
            double exposure_time = std::stod(exposure_time_str);

            std::string current_gray_file_path = "";
            std::string current_pcd_file_path = "";
            std::string current_rgb_file_path = "";

            // call service
            ros::ServiceClient client = root_nh_.serviceClient<utils_msgs::gwCameraTrigger>("gwCameraTrigger");
            utils_msgs::gwCameraTrigger srv;
            
            srv.request.exposure_time = exposure_time;
            srv.request.target_data_dir = target_data_dir;

            if(client.call(srv)){
                std::cout << "gwCameraTrigger success" << std::endl;
                current_gray_file_path = srv.response.current_data_dir + "/gray.bmp";
                current_pcd_file_path = srv.response.current_data_dir + "/point_cloud.pcd";
                current_rgb_file_path = srv.response.current_data_dir + "/rgb.bmp";
                res = srv.response.result;
            }
            else{
                print_warn("Failed to call service gwCameraTrigger");
                res = -1;
            }
            
            // 把数据赋值一份保存到本地
            std::string package_path = ros::package::getPath("arm_turntable_nbv_bt");
            // 创建 文件夹
            std::string gw_cmd = "mkdir -p " + package_path + "/data/" + cnt;
            system(gw_cmd.c_str());

            // 复制 文件到文件夹
            gw_cmd = "cp " + current_gray_file_path + " " + package_path + "/data/" + cnt + "/gray.bmp";
            system(gw_cmd.c_str());
            gw_cmd = "cp " + current_pcd_file_path + " " + package_path + "/data/" + cnt + "/point_cloud.pcd";
            system(gw_cmd.c_str());
            gw_cmd = "cp " + current_rgb_file_path + " " + package_path + "/data/" + cnt + "/rgb.bmp";
            system(gw_cmd.c_str());

            // set output
            setOutput("current_gray_file_path", package_path + "/data/" + cnt + "/gray.bmp");
            setOutput("current_pcd_file_path", package_path + "/data/" + cnt + "/point_cloud.pcd");
            setOutput("current_rgb_file_path", package_path + "/data/" + cnt + "/rgb.bmp");
            setOutput("current_dir", package_path + "/data/" + cnt);
            // 打印Log
            std::cout << "gwCameraTrigger log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "exposure_time: " << exposure_time << std::endl;
            std::cout << "target_data_dir: " << target_data_dir << std::endl;
            std::cout << "->OutputPorts " << std::endl;
            std::cout << "current_gray_file_path: " << package_path + "/data/" + cnt + "/gray.bmp" << std::endl;
            std::cout << "current_pcd_file_path: " << package_path + "/data/" + cnt + "/point_cloud.pcd" << std::endl;
            std::cout << "current_rgb_file_path: " << package_path + "/data/" + cnt + "/rgb.bmp" << std::endl;
            std::cout << "current_dir: " << package_path + "/data/" + cnt << std::endl;
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
