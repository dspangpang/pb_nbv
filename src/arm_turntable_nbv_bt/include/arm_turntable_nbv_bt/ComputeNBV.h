#pragma once

#include "./utils.h"
#include <utils_msgs/NBVTrigger.h>
#include <utils_msgs/GetPose.h>

namespace Turntable_NBV
{
    class ComputeNBV : public BT::SyncActionNode
    {
    public:
        ComputeNBV(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~ComputeNBV();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("pcd_file_path"));
            ports_list.emplace(BT::InputPort<std::string>("input_cnt"));

            ports_list.emplace(BT::OutputPort<std::string>("scanner_pose"));
            ports_list.emplace(BT::OutputPort<std::string>("is_terminated"));
            ports_list.emplace(BT::OutputPort<std::string>("output_cnt"));
            // add output ports
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "ComputeNBV is running " << this->name() << std::endl;

            // 获取输入参数
            auto pcd_file_path = getInput<std::string>("pcd_file_path").value();
            auto cnt = getInput<std::string>("input_cnt").value();

            // 结果信息
            int res = 0;
            // 变量转换
            int nbv_cnt = std::stoi(cnt);

            // 定义标准的旋转矩阵作为无旋转时的旋转矩阵
            Eigen::Matrix3d standard_rotation_matrix = Eigen::Matrix3d::Identity();          

            // call Getpose ros服务
            ros::ServiceClient client_pose = root_nh_.serviceClient<utils_msgs::GetPose>("GetPose");

            utils_msgs::GetPose srv_pose;
            srv_pose.request.link_name = "gw_scanner_depth_frame";
            geometry_msgs::Pose current_scanner_pose;
            if(client_pose.call(srv_pose)){
                current_scanner_pose = srv_pose.response.pose;
            }
            else{
                print_warn("Failed to call service GetPose");
                res = -1;
            }
            // 获取turntable support pose
            srv_pose.request.link_name = "turntable_support_link";
            geometry_msgs::Pose turntable_support_pose;
            if(client_pose.call(srv_pose)){
                turntable_support_pose = srv_pose.response.pose;
            }
            else{
                print_warn("Failed to call service GetPose");
                res = -1;
            }

            // 计算turntable_support_pose 相对于 standard_rotation_matrix 的 z 轴转角
            Eigen::Matrix4d turntable_support_matrix = poseToMatrix(turntable_support_pose);

            // 计算扫描仪的位姿沿着turntable_pose的z轴旋转rotation_rad后的位姿
            Eigen::Matrix4d current_scanner_matrix = poseToMatrix(current_scanner_pose);
            // 构造旋转轴和角度
            Eigen::Vector3d axis_point = turntable_support_matrix.block<3,1>(0,3);

            // 将位姿平移到轴的起点，进行旋转，然后再平移回来
            Eigen::Matrix4d translation_matrix1 = Eigen::Matrix4d::Identity();
            translation_matrix1.block<3,1>(0,3) = axis_point;

            Eigen::Matrix4d translation_matrix2 = Eigen::Matrix4d::Identity();
            translation_matrix2.block<3,1>(0,3) = -axis_point;

            Eigen::Matrix4d inverse_rotation_matrix = Eigen::Matrix4d::Identity();
            inverse_rotation_matrix.block<3,3>(0,0) = turntable_support_matrix.block<3,3>(0, 0).inverse();


            Eigen::Matrix4d true_scanner_matrix;

            true_scanner_matrix = translation_matrix1* 
                                    inverse_rotation_matrix * 
                                    translation_matrix2* current_scanner_matrix;

            // 把 true_scanner_matrix 写入到文件
            // 获取当前路径
            std::string package_path = ros::package::getPath("arm_turntable_nbv_bt");
            std::string true_scanner_matrix_file = package_path + "/data/true_scanner_matrix_" + std::to_string(nbv_cnt) + ".txt";
            std::ofstream out(true_scanner_matrix_file);
            if(out.is_open()){
                for(int i = 0; i < 4; i++){
                    for(int j = 0; j < 4; j++){
                        out << true_scanner_matrix(i,j) << " ";
                    }
                    out << std::endl;
                }
                out.close();
            }
            else{
                print_warn("Failed to open file");
                res = -1;
            }

            // true_scanner_matrix 转 pose 
            geometry_msgs::Pose true_scanner_pose = matrixToPose(true_scanner_matrix);

            // call NBVTrigger ros服务
            ros::ServiceClient client = root_nh_.serviceClient<utils_msgs::NBVTrigger>("mmr_ros_node_trigger");
            utils_msgs::NBVTrigger srv;
            srv.request.pcd_file_path = pcd_file_path;
            srv.request.current_camera_pose = true_scanner_pose;
            geometry_msgs::Pose nbv_scanner_pose;
            int is_terminated;
            if(client.call(srv)){
                nbv_scanner_pose = srv.response.nbv_camera_pose;
                is_terminated = srv.response.is_terminated;
                res = srv.response.result;
            }
            else{
                print_warn("Failed to call service NBVTrigger");
                res = -1;
            }
            
            // 输出参数
            setOutput("scanner_pose", poseToString(nbv_scanner_pose));
            setOutput("is_terminated", std::to_string(is_terminated));
            setOutput("output_cnt", std::to_string(nbv_cnt + 1));

            //打印信息
            std::cout << "ComputeNBV log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->current_scanner_pose " << poseToString(true_scanner_pose) << std::endl;
            std::cout << "--->pcd_file_path " << pcd_file_path << std::endl;
            std::cout << "--->cnt " << cnt << std::endl;
            std::cout << "->OutputPorts" << std::endl;
            std::cout << "--->scanner_pose " << poseToString(nbv_scanner_pose) << std::endl;
            std::cout << "--->is_terminated " << is_terminated << std::endl;
            std::cout << "--->output_cnt " << nbv_cnt + 1 << std::endl;
            std::cout << "res: " << res << std::endl;

            BT::NodeStatus status ;

            if(res == 0){
                status = BT::NodeStatus::SUCCESS;
                if (is_terminated == 1){
                    status = BT::NodeStatus::FAILURE;  // 此处的FAILURE代表任务结束
                }
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
