#pragma once

#include "./utils.h"

namespace Turntable_NBV
{
    class ComputeTurntableRotation : public BT::SyncActionNode
    {
    public:
        ComputeTurntableRotation(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~ComputeTurntableRotation();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;

            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("input_scanner_pose"));
            ports_list.emplace(BT::InputPort<std::string>("input_turntable_pose"));

            // add output ports
            ports_list.emplace(BT::OutputPort<std::string>("rotation_angle"));
            ports_list.emplace(BT::OutputPort<std::string>("scanner_pose"));
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "ComputeTurntableRotation is running " << this->name() << std::endl;

            // 获取输入参数
            auto input_scanner_pose = getInput<std::string>("input_scanner_pose").value();
            auto input_turntable_pose = getInput<std::string>("input_turntable_pose").value();
            
            // 结果信息
            int res = 0;

            // 变量转换
            geometry_msgs::Pose scanner_pose = analyseTargetPose(input_scanner_pose);
            geometry_msgs::Pose turntable_pose = analyseTargetPose(input_turntable_pose);

            Eigen::Matrix4d scanner_matrix = poseToMatrix(scanner_pose);
            Eigen::Matrix4d turntable_matrix = poseToMatrix(turntable_pose);


            // 计算扫描仪坐标系z轴与世界坐标系x轴的在xoy平面上的投影夹角
            // 取扫描仪坐标系z轴的向量
            Eigen::Vector3d scanner_z_axis = scanner_matrix.block<3,1>(0,2);
            // 计算夹角
            double rotation_rad = abs(atan2(scanner_z_axis(1), scanner_z_axis(0)));

            // 定义旋转的角度符号
            if(scanner_z_axis(1) >= 0){
                rotation_rad = -rotation_rad;
            }

            // 加入扰动
            rotation_rad += M_PI / 180 * 30;

            // 如果rad大于pi，则减去2pi
            if(rotation_rad > M_PI){
                rotation_rad -= 2 * M_PI;
            }else if(rotation_rad < -M_PI){
                rotation_rad += 2 * M_PI;
            }

            // 计算扫描仪的位姿沿着turntable_pose的z轴旋转rotation_rad后的位姿

            // 构造旋转轴和角度
            Eigen::Vector3d axis_point = turntable_matrix.block<3,1>(0,3);
            Eigen::Vector3d rotation_axis = Eigen::Vector3d(0,0,1);
            // 创建一个绕指定轴旋转的旋转矩阵
            Eigen::Quaterniond rotation(Eigen::AngleAxisd(rotation_rad, rotation_axis));
            Eigen::Matrix4d rotation_matrix = Eigen::Matrix4d::Identity();
            rotation_matrix.block<3,3>(0,0) = rotation.toRotationMatrix();

            // 将位姿平移到轴的起点，进行旋转，然后再平移回来
            Eigen::Matrix4d translation_matrix1 = Eigen::Matrix4d::Identity();
            translation_matrix1.block<3,1>(0,3) = axis_point;

            Eigen::Matrix4d translation_matrix2 = Eigen::Matrix4d::Identity();
            translation_matrix2.block<3,1>(0,3) = -axis_point;

            Eigen::Matrix4d new_scanner_matrix;
            new_scanner_matrix = translation_matrix1 * rotation_matrix * translation_matrix2 * scanner_matrix;

            // 设置output
            std::string out_scanner_pose_str = poseToString(matrixToPose(new_scanner_matrix));
            setOutput("scanner_pose", out_scanner_pose_str);
            setOutput("rotation_angle", std::to_string(rotation_rad * 180 / M_PI));
            

            std::cout << "ComputeTurntableRotation log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->input_scanner_pose " << input_scanner_pose << std::endl;
            std::cout << "--->input_turntable_pose " << input_turntable_pose << std::endl;
            std::cout << "->OutputPorts" << std::endl;
            std::cout << "rotation_angle: " << rotation_rad * 180 / M_PI << std::endl;
            std::cout << "scanner_pose: " << out_scanner_pose_str << std::endl;
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

        double hover_distance_;
    };

} // namespace Turntable_NBV
