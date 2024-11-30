#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <cfloat> // for DBL_MAX
#include <thread>

#include <glog/logging.h>

#include "utils.hpp"

class voxelstruct
{
private:

    // 点云参数
    std::vector<Eigen::Vector3d> occupied_voxels_;

    // voxel_map 参数
    double voxel_resolution_;
    Eigen::Vector3d bbx_unknown_min_;
    Eigen::Vector3d bbx_unknown_max_;
    double ray_trace_step_;
    int surrounding_voxels_radius_;

    // 相机参数
    std::vector<Eigen::Vector3d> frustum_points_;
    double camera_focal_length_;

public:
    voxelstruct();

    ~voxelstruct();

    /**
     * @brief 更新体素地图
     * @param cloud 输入点云
     * @param camera_pose 相机的位姿
     * @param output_frontier_voxels 输出的边界体素
     * @param output_occupied_voxels 输出的占据体素
     * @return octomap::ColorOcTree 体素地图
    */
    octomap::ColorOcTree update_voxel_map(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const Eigen::Matrix4d &camera_pose,
        std::vector<Eigen::Vector3d> &output_frontier_voxels,
        std::vector<Eigen::Vector3d> &output_occupied_voxels,
        Eigen::Vector3d &bbx_unknown_min_,
        Eigen::Vector3d &bbx_unknown_max);

private:
    /**
     * @brief 判断光线是否与包围盒相交
     * @param ray_origin 光线的起点
     * @param ray_direction 光线的方向
     * @param box_min voxel_map的包围盒最小值
     * @param box_max voxel_map的包围盒最大值
     * @param intersection 光线与包围盒的交点
     * @return bool 是否相交
    */
    bool computeRayBoxIntersection(
        const Eigen::Vector3d& ray_origin, 
        const Eigen::Vector3d& ray_direction, 
        const Eigen::Vector3d& box_min, 
        const Eigen::Vector3d& box_max, 
        Eigen::Vector3d& intersection);

    /**
     * @brief 计算光线穿过的体素的坐标
     * @param ray_start 光线的起点
     * @param ray_end 光线的终点
     * @return std::vector<Eigen::Vector3d> 光线穿过的体素的坐标
     * * J. Amanatides, A. Woo. A Fast Voxel Traversal Algorithm for Ray Tracing. Eurographics '87
    */
    std::vector<Eigen::Vector3d> ray_travel(const Eigen::Vector3d ray_start, const Eigen::Vector3d ray_end);
    
    /**
     * @brief 找到一个体素的所有邻居
    */
    std::vector<Eigen::Vector3d> find_neighbors(double x, double y, double z);

    /**
     * @brief 把Eigen::Vector3d转换成octomap::point3d
    */
    octomap::point3d to_oct3d(const Eigen::Vector3d &v);

    /**
     * @brief 把octomap::point3d转换成Eigen::Vector3d
    */
    Eigen::Vector3d to_eigen3d(const octomap::point3d &p);

    /**
     * @brief 获取frontier体素拓展区域,用于拓展包围盒
    */
    std::vector<Eigen::Vector3d> getSurroundingVoxels(const Eigen::Vector3d& voxel, int r);

    /**
     * @brief 清除包围盒外的体素
    */
    void clearOutsideBBX(
        octomap::ColorOcTree& map, 
        const octomap::point3d& min, 
        const octomap::point3d& max);
};

