#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <glog/logging.h>

#include <omp.h>

#include "utils.hpp"

class nbvstrategy
{
private:

    // 包围盒的位置
    Eigen::Vector3d bbx_unkonwn_min_;
    Eigen::Vector3d bbx_unkonwn_max_;

    // 第一帧视角位姿调整参数
    bool fist_viewpoint_ues_position_;
    double overlooking_angle_;
    double first_viewpoint_distance_factor_;
    Eigen::Vector3d first_viewpoint_position_;      

    // 随机生成视角的参数
    int random_candidate_points_num_;
    int longitude_num_;
    Eigen::MatrixXd candidate_longitude_angle_;
    Eigen::MatrixXd candidate_center_bias_;
    double partition_step_angle_size_;
    bool is_random_; // 如果为真，候选视角随机产生!!!!

    // 数据保存路径
    bool save_nbv_tmp_data_;
    std::string cluster_file_path_;
    std::string point_projection_file_path_;

    // nbv 策略相关参数
    int max_gmm_cluster_num_;
    int min_gmm_cluster_num_;

    // 相机参数解算
    Eigen::MatrixXd camera_intrinsic_;
    double camera_focal_length_;
    std::vector<Eigen::Vector3d> frustum_points_;

    // nbv 计数器
    int nbv_cnt_;

    // 中间数据保存
    std::pair<int, int> image_size_;
    std::vector<EllipsoidParam> ellipsoid_vec_;
    std::vector<Eigen::Matrix4d> camera_pose_vec_;

    // 分区状态
    std::vector<int> current_partition_state_;
    int left_index_;
    int right_index_;
    bool all_scanned_; 


public:
    nbvstrategy();

    ~nbvstrategy();

    /**
     * @brief 计算下一个视角
     * @param voxel_map 体素地图
     * @param current_camera_pose 当前相机位姿
     * @param current_frontier_voxels 当前边界体素
     * @param current_occupied_voxels 当前占据体素
     * @param bbx_unknown_min_ 体素地图的包围盒最小值
     * @param bbx_unknown_max 体素地图的包围盒最大值
     * @param nbv 下一个视角
     * @return 是否达到终止条件
    */
    int compute_next_view(
        const octomap::ColorOcTree &voxel_map,
        const Eigen::Matrix4d &current_camera_pose, 
        const std::vector<Eigen::Vector3d> &current_frontier_voxels,
        const std::vector<Eigen::Vector3d> &current_occupied_voxels,
        const Eigen::Vector3d &bbx_unknown_min,
        const Eigen::Vector3d &bbx_unknown_max,
        Eigen::Matrix4d& nbv);

    /**
     * @brief 获取聚类椭球参数
     * @return 是否达到终止条件
    */
   std::vector<EllipsoidParam> get_ellipsoids();

private:

    /**
     * @brief 计算初始视角
     * @return Eigen::Matrix4d 初始视角
    */
    Eigen::Matrix4d compute_init_view();

    /**
     * @brief 计算初始视角
     * @param camera_positon 相机位置
     * @return Eigen::Matrix4d 初始视角
    */
    Eigen::Matrix4d compute_init_view(const Eigen::Vector3d &camera_positon);

    /**
     * @brief 从候选视角中选择下一个视角
     * @param res 候选视角的评价结果
    */
    int nbv_selector(const std::vector<double>& res);
    
    /** 
     * @brief 计算cluster的最小外接椭球 使用CGAL库实现
     * @param frontier_voxels 聚类前沿的点
     * @param occupied_voxels 聚类占用体素
     * @return 0 成功
    */
    int cluster_ellipsoidization_CGAL(
        const std::vector<Eigen::Vector3d> &frontier_voxels,
        const std::vector<Eigen::Vector3d> &occupied_voxels);

    /** 
     * @brief 计算视角的投影
     * @param camera_pose 相机位姿
     * @param projection_img_out 投影图
     * @return 在该视角下的投影的面积
    */
    double cluster_projection(
        const Eigen::Matrix4d &camera_pose,
        cv::Mat &projection_img_out);

    /** 
     * @brief 计算视角的投影，使用 opencv 函数绘制椭圆
     * @param camera_pose 相机位姿
     * @param projection_img_out 投影图
     * @return 在该视角下的投影的面积
    */
    double cluster_projection_cv2(
        const Eigen::Matrix4d &camera_pose,
        cv::Mat &projection_img_out);

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
     * @brief 对 voxels 进行聚类 dbscan
     * @param voxels 输入体素
     * @return 聚类结果
    */
    std::vector<std::vector<Eigen::Vector3d>> 
    dbscan_clustering(const std::vector<Eigen::Vector3d> &voxels);
    
    /**
     * @brief 对 voxels 进行聚类 GMM
     * @param voxels 输入体素
     * @return 聚类结果
    */
    std::vector<std::vector<Eigen::Vector3d>> 
    gmm_clustering(const std::vector<Eigen::Vector3d> &voxels);

    /**
     * @brief 把Eigen::Vector3d转换成octomap::point3d
    */
    octomap::point3d to_oct3d(const Eigen::Vector3d &v);

    /**
     * @brief 把octomap::point3d转换成Eigen::Vector3d
    */
    Eigen::Vector3d to_eigen3d(const octomap::point3d &p);

    /**
     * @brief 根据椭球的参数计算椭球矩阵的对偶形式
     * @param param 椭球参数
     * @return 椭球矩阵的对偶形式
    */
    Eigen::Matrix4d create_ellipsoid_dual_matrix(const EllipsoidParam &param);

    /**
     * @brief 根据椭球的参数计算椭球矩阵
     * @param param 椭球参数
     * @return 椭球矩阵
    */
    Eigen::Matrix4d create_ellipsoid_matrix(const EllipsoidParam &param);

    /**
     * @brief 根据摄像机矩阵和椭球矩阵的对偶计算算椭圆矩阵
     * @param param 椭球参数
     * @return 椭球矩阵的对偶形式
    */
    Eigen::Matrix3d compute_ellipsoid_projection(
        const Eigen::Matrix<double, 3, 4> camera_matrix,
        const Eigen::Matrix4d ellipsoid_matrix_dual);


};

