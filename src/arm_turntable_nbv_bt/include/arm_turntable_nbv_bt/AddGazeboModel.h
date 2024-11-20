#pragma once

#include "./utils.h"
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include <streambuf>

namespace Turntable_NBV
{
    class AddGazeboModel : public BT::SyncActionNode
    {
    public:
        AddGazeboModel(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~AddGazeboModel();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("model_pose"));
            ports_list.emplace(BT::InputPort<std::string>("model_name"));
            ports_list.emplace(BT::InputPort<std::string>("model_ref_frame"));

            // add output ports
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "AddGazeboModel is running " << this->name() << std::endl;

            // 获取输入参数
            auto model_pose = getInput<std::string>("model_pose").value();
            auto model_name = getInput<std::string>("model_name").value();
            auto model_ref_frame = getInput<std::string>("model_ref_frame").value();

            // 结果信息
            int res = 0;

            // 变量转换
            geometry_msgs::Pose model_pose_msg = analyseTargetPose(model_pose);
            std::string model_file_path = "/root/work_place/building_editor_models/" + model_name + "/" + model_name + "_fp.sdf";

            // 通过ros向 gazebo 中添加模型
            ros::ServiceClient client = root_nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
            gazebo_msgs::SpawnModel srv;
            std::ifstream fin(model_file_path);
            std::string model_xml((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());

            srv.request.model_xml = model_xml;
            srv.request.model_name = model_name;
            srv.request.reference_frame = model_ref_frame;
            srv.request.initial_pose.position.x = model_pose_msg.position.x;
            srv.request.initial_pose.position.y = model_pose_msg.position.y;
            srv.request.initial_pose.position.z = model_pose_msg.position.z;
            srv.request.initial_pose.orientation.w = model_pose_msg.orientation.w;
            srv.request.initial_pose.orientation.x = model_pose_msg.orientation.x;
            srv.request.initial_pose.orientation.y = model_pose_msg.orientation.y;
            srv.request.initial_pose.orientation.z = model_pose_msg.orientation.z;

            if (client.call(srv))
            {
                res = 0;
            }
            else{
                print_warn("Failed to call service spawn_sdf_model");
                res = -1;
            }
            // 输出参数

            //打印信息
            std::cout << "AddGazeboModel log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->model_pose " << poseToString(model_pose_msg) << std::endl;
            std::cout << "--->model_name " << model_name << std::endl;
            std::cout << "--->model_ref_frame " << model_ref_frame << std::endl;
            std::cout << "->OutputPorts" << std::endl;
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
