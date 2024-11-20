#include "../include/arm_turntable_nbv_bt/ArmToTarget.h"
#include "../include/arm_turntable_nbv_bt/TurntableToTarget.h"
#include "../include/arm_turntable_nbv_bt/PubFrame.h"
#include "../include/arm_turntable_nbv_bt/AddObstacle.h"
#include "../include/arm_turntable_nbv_bt/ComputeNBV.h"
#include "../include/arm_turntable_nbv_bt/GetPointCloud.h"
#include "../include/arm_turntable_nbv_bt/ComputeFrameTrans.h"
#include "../include/arm_turntable_nbv_bt/AddGazeboModel.h"
#include "../include/arm_turntable_nbv_bt/MoveGazeboModel.h"
#include "../include/arm_turntable_nbv_bt/ComputeTurntableRotation.h"
#include "../include/arm_turntable_nbv_bt/CheckFirstFrameCondition.h"
#include "../include/arm_turntable_nbv_bt/CheckTerminatedCondition.h"
#include "../include/arm_turntable_nbv_bt/gwCameraTrigger.h"
#include "../include/arm_turntable_nbv_bt/SegmentedPose.h"
#include "../include/arm_turntable_nbv_bt/PcpTrigger.h"
#include "../include/arm_turntable_nbv_bt/WaitKey.h"
#include "../include/arm_turntable_nbv_bt/GetPose.h"


#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "ros/package.h"

int main(int argc, char ** argv){

    ros::init(argc, argv, "compute_template_node");

    ros::NodeHandle root_nh;

    ros::NodeHandle bh_tree_nh("~");

    BT::BehaviorTreeFactory factory;

    // 清空 data 文件夹 中所有 pcd 文件
    std::string data_path = ros::package::getPath("arm_turntable_nbv_bt") + "/data";
    std::string cmd = "rm -rf " + data_path + "/*";
    system(cmd.c_str());


    // add ArmToTarget node
    BT::NodeBuilder builder_ArmToTarget = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::ArmToTarget>(name, config, root_nh, bh_tree_nh);
    };

    factory.registerBuilder<Turntable_NBV::ArmToTarget>("ArmToTarget", builder_ArmToTarget);

    // add AddObstacle node
    BT::NodeBuilder builder_AddObstacle = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::AddObstacle>(name, config, root_nh, bh_tree_nh);
    };

    factory.registerBuilder<Turntable_NBV::AddObstacle>("AddObstacle", builder_AddObstacle);

    // add PubFrame node
    BT::NodeBuilder builder_PubFrame = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::PubFrame>(name, config, root_nh, bh_tree_nh);
    };

    factory.registerBuilder<Turntable_NBV::PubFrame>("PubFrame", builder_PubFrame);


    // add TurntableToTarget node
    BT::NodeBuilder builder_TurntableToTarget = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::TurntableToTarget>(name, config, root_nh, bh_tree_nh);
    };

    factory.registerBuilder<Turntable_NBV::TurntableToTarget>("TurntableToTarget", builder_TurntableToTarget);
    
    // add ComputeNBV node
    BT::NodeBuilder builder_ComputeNBV = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::ComputeNBV>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::ComputeNBV>("ComputeNBV", builder_ComputeNBV);

    // add GetPointCloud node
    BT::NodeBuilder builder_GetPointCloud = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::GetPointCloud>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::GetPointCloud>("GetPointCloud", builder_GetPointCloud);

    // add ComputeFrameTrans node
    BT::NodeBuilder builder_ComputeFrameTrans = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::ComputeFrameTrans>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::ComputeFrameTrans>("ComputeFrameTrans", builder_ComputeFrameTrans);

    // add AddGazeboModel node
    BT::NodeBuilder builder_AddGazeboModel = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::AddGazeboModel>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::AddGazeboModel>("AddGazeboModel", builder_AddGazeboModel);

    // add MoveGazeboModel node
    BT::NodeBuilder builder_MoveGazeboModel = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::MoveGazeboModel>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::MoveGazeboModel>("MoveGazeboModel", builder_MoveGazeboModel);

    // add ComputeTurntableRotation node
    BT::NodeBuilder builder_ComputeTurntableRotation = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::ComputeTurntableRotation>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::ComputeTurntableRotation>("ComputeTurntableRotation", builder_ComputeTurntableRotation);

    // add CheckFirstFrameCondition node
    BT::NodeBuilder builder_CheckFirstFrameCondition = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::CheckFirstFrameCondition>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::CheckFirstFrameCondition>("CheckFirstFrameCondition", builder_CheckFirstFrameCondition);

    // add CheckTerminatedCondition node
    BT::NodeBuilder builder_CheckTerminatedCondition = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::CheckTerminatedCondition>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::CheckTerminatedCondition>("CheckTerminatedCondition", builder_CheckTerminatedCondition);

    // add gwCameraTrigger node
    BT::NodeBuilder builder_gwCameraTrigger = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::gwCameraTrigger>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::gwCameraTrigger>("gwCameraTrigger", builder_gwCameraTrigger);

    // add SegmentedPose node
    BT::NodeBuilder builder_SegmentedPose = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::SegmentedPose>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::SegmentedPose>("SegmentedPose", builder_SegmentedPose);

    // add PcpTrigger node
    BT::NodeBuilder builder_PcpTrigger = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::PcpTrigger>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::PcpTrigger>("PcpTrigger", builder_PcpTrigger);

    // add WaitKey node
    BT::NodeBuilder builder_WaitKey = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::WaitKey>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::WaitKey>("WaitKey", builder_WaitKey);

    // add GetPose node
    BT::NodeBuilder builder_GetPose = [&root_nh, &bh_tree_nh](const std::string& name,
                                                                const BT::NodeConfiguration& config) {
        return std::make_unique<Turntable_NBV::GetPose>(name, config, root_nh, bh_tree_nh);
    };
    factory.registerBuilder<Turntable_NBV::GetPose>("GetPose", builder_GetPose);

    // construct tree 
    auto file_path = ros::package::getPath("arm_turntable_nbv_bt") + "/config/tree/ComputeTemplate.xml";

    auto tree = factory.createTreeFromFile(file_path);

    BT::PublisherZMQ publisher_zmq(tree);
    
    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    status = tree.tickRoot();

    ros::spin();
}


