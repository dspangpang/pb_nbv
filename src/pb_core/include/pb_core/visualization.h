#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <glog/logging.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <thread>
#include <mutex>

#include "utils.hpp"


class visualization
{
private:
    bool window_should_close_;
    bool update_flag_;
    std::mutex my_mutex_;

    cv::viz::Viz3d viz_;

private:
    Eigen::Vector3d bbx_unknown_min_;
    Eigen::Vector3d bbx_unknown_max_;

    std::vector<Eigen::Matrix4d> camera_pose_;
    double camera_focal_length_;

    std::vector<Eigen::Vector3d> frustum_points_;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3i>> voxel_map_;

    double voxel_resolution_;

    std::vector<EllipsoidParam> ellipsoid_vec_;

    int random_candidate_points_num_;
    int longitude_num_;
    Eigen::MatrixXd candidate_longitude_angle_;
    Eigen::MatrixXd candidate_center_bias_;
    
    Eigen::Vector3d candidate_points_min_;
    int current_view_cnt_ = 0;
    int total_view_cnt_ = 0;
    std::vector<Eigen::Matrix4d> candidate_points_; 


private:
    bool show_frustum_;
    bool show_voxel_map_;
    bool show_ellipsoid_;
    bool show_ray_;
    bool show_candidate_points_;
    bool show_candidate_frames_;
    bool show_bbx_;
    bool show_best_view_frames_;

public:
    visualization();

    ~visualization();
    
    /**
     * @brief 捕获一帧数据
     * @param camera_pose 相机的位姿
     * @param voxel_map 体素地图
     * @param bbx_unknown_min 未知体素的包围盒最小值
     * @param bbx_unknown_max 未知体素的包围盒最大值
    */
    void catpure_frame(
        const Eigen::Matrix4d &camera_pose,
        const octomap::ColorOcTree &voxel_map_tree,
        const Eigen::Vector3d &bbx_unknown_min,
        const Eigen::Vector3d &bbx_unknown_max);

    /**
     * @brief 捕获一帧数据
     * @param camera_pose 相机的位姿
     * @param ellipsoid_vec 椭球参数
     * @param voxel_map 体素地图
     * @param bbx_unknown_min 未知体素的包围盒最小值
     * @param bbx_unknown_max 未知体素的包围盒最大值
    */
    void catpure_frame(
        const Eigen::Matrix4d &camera_pose,
        const std::vector<EllipsoidParam> ellipsoid_vec,
        const octomap::ColorOcTree &voxel_map_tree,
        const Eigen::Vector3d &bbx_unknown_min,
        const Eigen::Vector3d &bbx_unknown_max);

    /**
     * @brief 添加线集
     * @param lineset 线集 
    */
    void add_lineset(const std::vector<cv::viz::WLine> &lineset);

    /**
     * @brief 可视化结果
    */
    void visualize_result();

private:

    /**
     * @brief 设置更新标志
     * @param update_flag 更新标志
    */
    void set_update_flag(const bool &update_flag);
    /**
     * @brief 获取更新标志
     * @return bool 更新标志
    */
    bool get_update_flag();

    /**
     * @brief 设置关闭标志
     * @param close_flag 关闭标志
    */
    void set_close_flag(const bool &close_flag);
    /**
     * @brief 获取关闭标志
     * @return bool 关闭标志
    */
    bool get_close_flag();

    /**
     * @brief 获取未知体素的包围盒
     * @return std::vector<cv::viz::WLine> 包围盒
    */
    std::vector<cv::viz::WLine> get_unknown_bbx();

    /**
     * @brief 获取相机的视锥
     * @param depth 视锥的深度
     * @param frustum_pose 视锥的位姿
     * @return std::vector<cv::viz::WLine> 视锥
    */
    std::vector<cv::viz::WLine> get_frustum(
        const double depth, 
        const Eigen::Matrix4d frustum_pose);

    /**
     * @brief 遍历candidate_point更新viz的观测视角
    */
    void update_viz_views();
};

