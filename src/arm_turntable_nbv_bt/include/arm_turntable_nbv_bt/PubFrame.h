#pragma once

#include "./utils.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

namespace Turntable_NBV
{
    class PubFrame : public BT::SyncActionNode
    {
    public:
        PubFrame(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~PubFrame();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("parent_frame"));
            ports_list.emplace(BT::InputPort<std::string>("target_frame"));
            ports_list.emplace(BT::InputPort<std::string>("target_pose"));

            // Add output ports
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "PubFrame is running " << this->name() << std::endl;

            // 获取输入参数
            auto parent_frame = getInput<std::string>("parent_frame").value();
            
            auto target_frame = getInput<std::string>("target_frame").value();

            auto target_pose_str = getInput<std::string>("target_pose").value();

            // 结果信息
            int res = 0;

            
            // 变量转换
            geometry_msgs::Pose target_pose = analyseTargetPose(target_pose_str);

            // pub frame
            // 创建 tf2 的广播对象
            static tf2_ros::StaticTransformBroadcaster static_broadcaster;

            // 创建 tf2 要广播的静态坐标变换
            geometry_msgs::TransformStamped static_transform_stamped;

            // 对坐标变换初始化
            static_transform_stamped.header.stamp = ros::Time::now();
            // 父节点
            static_transform_stamped.header.frame_id = parent_frame;
            // 子节点
            static_transform_stamped.child_frame_id = target_frame;

            static_transform_stamped.transform.translation.x = target_pose.position.x;
            static_transform_stamped.transform.translation.y = target_pose.position.y;
            static_transform_stamped.transform.translation.z = target_pose.position.z;
            static_transform_stamped.transform.rotation.w = target_pose.orientation.w;
            static_transform_stamped.transform.rotation.x = target_pose.orientation.x;
            static_transform_stamped.transform.rotation.y = target_pose.orientation.y;
            static_transform_stamped.transform.rotation.z = target_pose.orientation.z;

            static_broadcaster.sendTransform(static_transform_stamped);

            // 打印Log
            std::cout << "PubFrame log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->parent_frame " << parent_frame << std::endl;
            std::cout << "--->target_frame " << target_frame << std::endl;
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


