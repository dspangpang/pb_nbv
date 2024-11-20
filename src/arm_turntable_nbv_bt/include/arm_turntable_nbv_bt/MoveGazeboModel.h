#pragma once

#include "./utils.h"
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/SetModelState.h>
#include <fstream>
#include <streambuf>

namespace Turntable_NBV
{
    class MoveGazeboModel : public BT::AsyncActionNode
    {
    public:
        MoveGazeboModel(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~MoveGazeboModel();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("model_name"));
            ports_list.emplace(BT::InputPort<std::string>("model_ref_frame"));
            // add output ports
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "MoveGazeboModel is running " << this->name() << std::endl;

            // 获取输入参数
            auto model_name = getInput<std::string>("model_name").value();
            auto model_ref_frame = getInput<std::string>("model_ref_frame").value();

            // 结果信息
            int res = 0;

            // 变量转换

            // 通过ros向 gazebo 修改模型位置
            ros::ServiceClient client_gazebo = root_nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
            // ros 查询某个link的位姿
            ros::ServiceClient client_pose = root_nh_.serviceClient<utils_msgs::GetPose>("GetPose");

            bool shuld_exit = false;

            utils_msgs::GetPose srv;
            srv.request.link_name = model_ref_frame;

            // 启动一个线程倒计时5s
            std::thread t1([&](){
                std::this_thread::sleep_for(std::chrono::seconds(6));
                shuld_exit = true;
            });
            t1.detach();            

            while(ros::ok() && !shuld_exit){
                if(client_pose.call(srv)){
                    auto pose = srv.response.pose;

                    gazebo_msgs::SetModelState srv;
                    srv.request.model_state.model_name = model_name;
                    srv.request.model_state.pose = pose;
            
                    if (client_gazebo.call(srv))
                    {
                        res = 0;
                    }
                    else{
                        print_warn("Failed to call service spawn_sdf_model");
                        res = -1;
                    }
                }
                else{
                    print_warn("Failed to call service GetPose");
                    res = -1;
                }
            }
            // 输出参数

            //打印信息
            std::cout << "MoveGazeboModel log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
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
