#pragma once

#include "./utils.h"
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

namespace Turntable_NBV
{
    class AddObstacle : public BT::SyncActionNode
    {
    public:
        AddObstacle(const std::string& name, 
                    const BT::NodeConfiguration& config, 
                    const ros::NodeHandle& root_nh,
                    const ros::NodeHandle& tree_nh);

        ~AddObstacle();

        // 处理接口参数
        static BT::PortsList providedPorts() {
            BT::PortsList ports_list;
            
            // Add input ports
            ports_list.emplace(BT::InputPort<std::string>("obstacle_name"));
            ports_list.emplace(BT::InputPort<std::string>("parent_frame"));
            ports_list.emplace(BT::InputPort<std::string>("obstacle_shape"));
            ports_list.emplace(BT::InputPort<std::string>("obstacle_pose"));
            ports_list.emplace(BT::InputPort<std::string>("obstacle_size"));

            // Add output ports
            
            return ports_list;
        }

        // You must override the virtual function tick()
        BT::NodeStatus tick() override
        {   
            std::cout << "AddObstacle is running " << this->name() << std::endl;

            // 获取参数
            std::string name = getInput<std::string>("obstacle_name").value();
            std::string parent_frame = getInput<std::string>("parent_frame").value();
            std::string shape = getInput<std::string>("obstacle_shape").value();
            std::string pose_str = getInput<std::string>("obstacle_pose").value();
            std::string size_str = getInput<std::string>("obstacle_size").value();

            // 结果信息
            int res = 0;

            // 变量转换
            geometry_msgs::Pose pose = analyseTargetPose(pose_str);
            std::vector<double> size = analyseJointValue(size_str);

            // call service
            
            // 创建PlanningScene服务的客户端
            ros::ServiceClient planning_scene_client = 
                                    root_nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");

            // 创建障碍物对象
            moveit_msgs::CollisionObject obstacle;
            obstacle.id = name; // 障碍物的唯一标识符
            obstacle.header.frame_id = parent_frame; // 障碍物的参考坐标系

            // 定义障碍物的形状和位置
            shape_msgs::SolidPrimitive obstacle_shape;
            if (shape == "sphere")
            {
                obstacle_shape.type = shape_msgs::SolidPrimitive::SPHERE; // 障碍物形状（例如：球体）
            }else if (shape == "cylinder")
            {
                obstacle_shape.type = shape_msgs::SolidPrimitive::CYLINDER; // 障碍物形状（例如：柱体）
            }else if (shape == "box")
            {
                obstacle_shape.type = shape_msgs::SolidPrimitive::BOX; // 障碍物形状（例如：柱体
            }else
            {
                std::cout << "shape error" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            obstacle_shape.dimensions.resize(size.size());
            obstacle_shape.dimensions[0] = 0.1; 
            for (size_t i = 0; i < size.size(); i++)
            {
                obstacle_shape.dimensions[i] = size[i]; // 障碍物的尺寸
            }
            
            // 定义障碍物的位置
            geometry_msgs::Pose obstacle_pose = pose;

            obstacle.primitives.push_back(obstacle_shape);
            obstacle.primitive_poses.push_back(obstacle_pose);
            obstacle.operation = moveit_msgs::CollisionObject::ADD;

            // 构建PlanningScene消息
            moveit_msgs::PlanningScene planning_scene;
            planning_scene.world.collision_objects.push_back(obstacle);
            planning_scene.is_diff = true; // 将is_diff字段设置为true表示只更新差异部分

            // 调用Planning Scene服务
            moveit_msgs::ApplyPlanningScene srv;
            srv.request.scene = planning_scene;

            if (planning_scene_client.call(srv)) {
                res = 0;
            } else {
                res = -1;
            }
            
            // 打印Log
            std::cout << "AddObstacle log : " << this->name() << std::endl;
            std::cout << "->InputPorts" << std::endl;
            std::cout << "--->name " << name << std::endl;
            std::cout << "--->parent_frame " << parent_frame << std::endl;
            std::cout << "--->shape " << shape << std::endl;
            std::cout << "--->pose_str " << pose_str << std::endl;
            std::cout << "--->size_str " << size_str << std::endl;
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
