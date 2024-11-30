#include "../include/pb_core/pbnbv.h"

pbnbv::pbnbv():is(),ns(),vs()
{
    currnet_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    current_camera_pose = Eigen::Matrix4d::Identity();

    std::string config_file_path = "/root/work_place/pb_nbv/src/pb_core/config/config.json";

    // 计算初始的包围盒的位置
    data_root_path_ = parseJsonString(config_file_path, "data_root_path");
    LOG(INFO) << "pbnbv init success";

    // 是否开启可视化
    enable_visualization_ = parseJsonBool(config_file_path, "enable_visualization");
    LOG(INFO) << "enable_visualization: " << enable_visualization_;

    // 是否完成
    is_done_ = false;
}

pbnbv::~pbnbv() = default;

void pbnbv::capture(        
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const Eigen::Matrix4d &camera_pose)
{
    currnet_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 深拷贝
    pcl::copyPointCloud(*cloud, *currnet_cloud);
    current_camera_pose = camera_pose;
}

int pbnbv::execute(Eigen::Matrix4d & nbv)
{   
    LOG (INFO) << "execute :";
    
    std::vector<Eigen::Vector3d> current_frontier_voxels;
    std::vector<Eigen::Vector3d> current_occupied_voxels;
    Eigen::Vector3d bbx_unknown_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbx_unknown_max = Eigen::Vector3d::Zero();
    current_frontier_voxels.clear();
    current_occupied_voxels.clear();
    
    // 计算耗时 NBV 规划器耗时
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    LOG (INFO) << "update_voxel_map : ";
    auto voxel_map = is.update_voxel_map(
        currnet_cloud, 
        current_camera_pose, 
        current_frontier_voxels, 
        current_occupied_voxels,
        bbx_unknown_min,
        bbx_unknown_max);

    this->voxel_map_size_ = voxel_map.getNumLeafNodes();
    LOG(INFO) << "voxel_map size: " << voxel_map.getNumLeafNodes();

    LOG (INFO) << "compute_next_view : ";
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    int res = ns.compute_next_view(
        voxel_map, 
        current_camera_pose,
        current_frontier_voxels,
        current_occupied_voxels,
        bbx_unknown_min,
        bbx_unknown_max,
        nbv);
    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    LOG(INFO) << "out nbv: " << "\r\n" << nbv;
    LOG(INFO) << "execute time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count() << " ms";
    LOG(INFO) << "update_voxel_map time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms";
    LOG(INFO) << "compute_next_view time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms";

    // 把 execute time 保存到文件
    // std::ofstream time_file;
    // time_file.open(data_root_path_ + "compute_time.txt", std::ios::app);
    // time_file << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t1).count() << "\r\n";
    // time_file.close();

    this->ellipsoid_size_ = ns.get_ellipsoids().size();
    vs.catpure_frame(nbv, ns.get_ellipsoids(), voxel_map, bbx_unknown_min, bbx_unknown_max);
    
    if (res == 1){
        LOG (INFO) << "Iteration Terminated!";
        is_done_ = true;
    }

    return res;
}

void pbnbv::visualization_start()
{
    if (enable_visualization_)
    {
        vs.visualize_result();
    }else{
        LOG(INFO) << "visualization is not enabled";
        while (!is_done_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    
}