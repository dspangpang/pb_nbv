#pragma once

#include "./utils.h"

namespace Turntable_NBV
{
    class ComputeFrameTrans : public BT::SyncActionNode
    {
    public:
        ComputeFrameTrans(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~ComputeFrameTrans();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("known_frame_name"));
            ports_list.emplace(BT::InputPort<std::string>("known_frame_pose"));
            ports_list.emplace(BT::InputPort<std::string>("target_frame_name"));

            ports_list.emplace(BT::OutputPort<std::string>("target_frame_pose"));
            // add output ports
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "ComputeFrameTrans is running " << this->name() << std::endl;

            // 获取输入参数
            auto known_frame_name = getInput<std::string>("known_frame_name").value();
            auto known_frame_pose = getInput<std::string>("known_frame_pose").value();
            auto target_frame_name = getInput<std::string>("target_frame_name").value();


            // 结果信息
            int res = 0;

            // 变量转换
            geometry_msgs::Pose known_pose = analyseTargetPose(known_frame_pose);
            geometry_msgs::Pose target_frame_pose;
            // tf 获得两位姿之间的变换
            tf::TransformListener listener;
            // 等待变换可用
            try {
                listener.waitForTransform(known_frame_name, target_frame_name, ros::Time(0), ros::Duration(10.0));
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }

            // 查找变换
            tf::StampedTransform ts;
            try {
                listener.lookupTransform(known_frame_name, target_frame_name, ros::Time(0), ts);
            } catch (tf::TransformException &ex) {
                ROS_ERROR("%s", ex.what());
            }
            // 通过变换获得目标位姿
            // 计算known_pose的齐次变化矩阵
            Eigen::Matrix4d known_pose_mat = poseToMatrix(known_pose);
            // 计算ts的齐次变化矩阵
            geometry_msgs::Pose ts_p;
            ts_p.position.x = ts.getOrigin().x();
            ts_p.position.y = ts.getOrigin().y();
            ts_p.position.z = ts.getOrigin().z();
            ts_p.orientation.x = ts.getRotation().x();
            ts_p.orientation.y = ts.getRotation().y();
            ts_p.orientation.z = ts.getRotation().z();
            ts_p.orientation.w = ts.getRotation().w();
            Eigen::Matrix4d ts_mat = poseToMatrix(ts_p);
            // 计算target_pose的齐次变换矩阵
            Eigen::Matrix4d target_pose_mat = known_pose_mat * ts_mat;
            target_frame_pose = matrixToPose(target_pose_mat);

            // 输出参数
            setOutput("target_frame_pose", poseToString(target_frame_pose));

            //打印信息
            std::cout << "ComputeFrameTrans log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->known_frame_name " << known_frame_name << std::endl;
            std::cout << "--->known_frame_pose " << known_frame_pose << std::endl;
            std::cout << "--->target_frame_name " << target_frame_name << std::endl;
            std::cout << "->OutputPorts" << std::endl;
            std::cout << "--->target_frame_pose " << poseToString(target_frame_pose) << std::endl;
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
