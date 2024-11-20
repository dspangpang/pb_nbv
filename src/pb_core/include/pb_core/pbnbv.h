#pragma once

#include "voxelstruct.h"
#include "jsonparser.hpp"
#include "nbvstrategy.h"
#include "visualization.h"
#include "utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

class pbnbv{

public:

    pbnbv();

    ~pbnbv();

    void capture(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const Eigen::Matrix4d &camera_pose);

    /**
     * @brief 执行下一个视角
     * @param nbv 下一个视角
     * @return 是否达到终止条件 1: 达到终止条件 0: 未达到终止条件
    */
    int execute(Eigen::Matrix4d & nbv);

    void visualization_start();

public:
  
private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr currnet_cloud;
    Eigen::Matrix4d current_camera_pose;
    std::string data_root_path_;
    bool enable_visualization_;
    bool is_done_;

private:
    voxelstruct is;
    nbvstrategy ns;
    visualization vs;
};