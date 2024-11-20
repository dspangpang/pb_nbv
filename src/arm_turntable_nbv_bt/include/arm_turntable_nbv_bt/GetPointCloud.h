#pragma once

#include "./utils.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace Turntable_NBV
{
    class GetPointCloud : public BT::SyncActionNode
    {
    public:
        GetPointCloud(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~GetPointCloud();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("point_cloud_topic"));
            ports_list.emplace(BT::InputPort<std::string>("cnt"));

            // Add output 
            ports_list.emplace(BT::OutputPort<std::string>("current_dir"));
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "GetPointCloud is running " << this->name() << std::endl;

            // 获取输入参数
            auto point_cloud_topic = getInput<std::string>("point_cloud_topic").value();
            auto cnt = getInput<std::string>("cnt").value();

            // 结果信息
            int res = 0;

            // 等待获取一帧点云消息
            sensor_msgs::PointCloud2::ConstPtr 
            point_cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>
                                (point_cloud_topic, root_nh_);

            // 转换为pcl点云并保存到本地
            pcl::PointCloud<pcl::PointXYZ>::Ptr
            point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

            // 把数据赋值一份保存到本地
            std::string package_path = ros::package::getPath("arm_turntable_nbv_bt");
            // 创建 文件夹
            std::string gw_cmd = "mkdir -p " + package_path + "/data/" + cnt;
            system(gw_cmd.c_str());

            // 保存点云文件到文件夹
            
            std::string point_cloud_path = package_path + "/data/" + cnt +"/point_cloud.pcd";
            pcl::io::savePCDFileBinary(point_cloud_path, *point_cloud);
            setOutput("current_dir", package_path + "/data/" + cnt);

            
            std::cout << "GetPointCloud log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "point_cloud_topic " << point_cloud_topic << std::endl;
            std::cout << "cnt " << cnt << std::endl;
            std::cout << "->OutputPorts " << std::endl;
            std::cout << "current_dir " << package_path + "/data/" + cnt << std::endl;
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
