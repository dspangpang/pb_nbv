#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include <filesystem>

#include "jsonparser.hpp"
#include <chrono>

struct EllipsoidParam
{
    std::string type;
    Eigen::Matrix4d pose;
    Eigen::Vector3d radii;
};

inline void computeInitBBX(
    const Eigen::Matrix4d& object_pose,
    Eigen::Vector3d& bbx_unknown_min,
    Eigen::Vector3d& bbx_unknown_max,
    const double & unknown_voxel_bbx_side = 0.3){

    Eigen::Vector3d tmp_min = Eigen::Vector3d(-unknown_voxel_bbx_side / 2.0, 
                                            -unknown_voxel_bbx_side / 2.0, 
                                            -unknown_voxel_bbx_side / 2.0);

    Eigen::Vector3d tmp_max = Eigen::Vector3d(unknown_voxel_bbx_side / 2.0, 
                                            unknown_voxel_bbx_side / 2.0, 
                                            unknown_voxel_bbx_side / 2.0);

    bbx_unknown_min = object_pose.block<3,3>(0,0) * tmp_min + object_pose.block<3,1>(0,3);
    bbx_unknown_max = object_pose.block<3,3>(0,0) * tmp_max + object_pose.block<3,1>(0,3);
}


inline void analyzeCameraIntrinsic(
    const Eigen::Matrix3d& camera_intrinsic,
    const double &camera_focal_length_factor,
    std::vector<Eigen::Vector3d>& frustum_points,
    double &camera_focal_length,
    std::pair<int, int>& image_size){
    
    frustum_points.resize(4);
    // 计算相机的焦距
    auto camera_focal_length_x = camera_intrinsic(0, 0);
    auto camera_focal_length_y = camera_intrinsic(1, 1);
    // 提取图像中心
    auto camera_principal_point_x = camera_intrinsic(0, 2);
    auto camera_principal_point_y = camera_intrinsic(1, 2);
    // 计算图像的宽度和高度
    auto camera_image_width_d = 2 * camera_principal_point_x;
    auto camera_image_height_d = 2 * camera_principal_point_y;
    // 计算焦距的物理尺寸
    auto camera_focal_length_x_m = camera_focal_length_x / 1000;
    auto camera_focal_length_y_m = camera_focal_length_y / 1000;
    // 计算视锥的四个顶点
    frustum_points[0] = Eigen::Vector3d(
        (- camera_image_width_d / 2) / camera_focal_length_x * 1.0, 
        (- camera_image_height_d / 2) / camera_focal_length_y * 1.0, 
        1.0);
    frustum_points[1] = Eigen::Vector3d(
        (camera_image_width_d / 2) / camera_focal_length_x * 1.0, 
        ( - camera_image_height_d / 2) / camera_focal_length_y * 1.0, 
        1.0);
    frustum_points[2] = Eigen::Vector3d(
        (camera_image_width_d / 2) / camera_focal_length_x * 1.0, 
        (camera_image_height_d / 2) / camera_focal_length_y * 1.0, 
        1.0);
    frustum_points[3] = Eigen::Vector3d(
        (- camera_image_width_d / 2) / camera_focal_length_x * 1.0,
        (camera_image_height_d / 2) / camera_focal_length_y * 1.0,
        1.0);

    camera_focal_length = (camera_focal_length_x_m+camera_focal_length_y_m) / 2 * camera_focal_length_factor;
    image_size.first = int(camera_image_width_d);
    image_size.second = int(camera_image_height_d);
}

/**
 * @brief 生成候选视角
 * @return std::vector<Eigen::Matrix4d> 候选视角
*/
inline std::vector<Eigen::Matrix4d> generate_candidate_views(
    const Eigen::Vector3d& bbx_unknown_min,
    const Eigen::Vector3d& bbx_unknown_max,
    const double &camera_focal_length,
    const int &random_candidate_points_num,
    const int &longitude_num,
    const double &longitude_step_upper_bound = 90.0,
    const double &longitude_step_lower_bound = 40.0,
    const Eigen::Vector3d &center_bias = Eigen::Vector3d(0,0,0)){

    std::vector<Eigen::Matrix4d> candidate_view_pose;

    // 生成一个球体，球心为目标点云的质心，半径为目标点云的半径 + 相机focal_length
    // 计算球体的半径
    double radius = (bbx_unknown_max - bbx_unknown_min).norm() / 2.0 + camera_focal_length;
    // 计算未知bbx的中心
    Eigen::Vector3d center = (bbx_unknown_max + bbx_unknown_min) / 2.0;
    center = center + center_bias;

    std::vector<Eigen::Vector3d> candidate_points;

    // 均匀分布策略是 把纬度分为x份，每个纬度上的点的数量按照对应的经线的长度来分配
    // 生成点的总和为最接近 random_candidate_points_num 的整数

    // 0度对应北极 90度对应赤道 180度对应南极
    // 只采用上半球的[40-90]度 60度的范围
    double longitude_length = 0;
    double start_longitude = M_PI / 180 * longitude_step_lower_bound;
    double end_longitude = M_PI / 180 * longitude_step_upper_bound;
    double latitude_step = (end_longitude - start_longitude) / longitude_num;

    for(int i = 0; i < longitude_num; i++){
        // 计算每个纬度上的经线的总长度
        double latitude = start_longitude + latitude_step * i;
        longitude_length += 2 * M_PI * sin(latitude) * radius;
    }
    // 计算每个点在经线上的间隔
    double longitude_arc = longitude_length / random_candidate_points_num;

    for(int i = 0; i < longitude_num; i++){
        // 维度为 [40，90]只有上半球
        double latitude = start_longitude + latitude_step * i;
        // 计算每个纬度上的经线的总长度
        double local_radius = 2 * M_PI * sin(latitude) * radius;
        // 计算每个点在经线上的数量
        int local_points_num = round(local_radius / longitude_arc);
        for (int j = 0; j < local_points_num; j++){
            double longitude = 2 * M_PI * j / local_points_num;
            Eigen::Vector3d random_point = Eigen::Vector3d(radius * sin(latitude) * cos(longitude),
                                                            radius * sin(latitude) * sin(longitude),
                                                            radius * cos(latitude)) + center;
            candidate_points.push_back(random_point);
        }
    }
    
    // 遍历所有的随机点，计算每个点的视角,每个点视角的z轴都指向目标点云的质心
    for(size_t i = 0; i < candidate_points.size(); i++){
        Eigen::Vector3d z_axis = center - candidate_points[i];
        z_axis.normalize();
        // x轴的方向与z轴在水平面垂直
        Eigen::Vector3d x_axis = z_axis.cross(Eigen::Vector3d::UnitZ());
        x_axis.normalize();
        // y轴的方向与z轴和x轴垂直, x,y,z轴构成右手坐标系
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        // 保证z轴方向与x,y轴正交
        z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // 计算旋转矩阵
        Eigen::Matrix3d rotation;
        rotation << x_axis, y_axis, z_axis;

        Eigen::Vector3d position = candidate_points[i];
        
        // 构建齐次变换矩阵
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3,3>(0,0) = rotation;
        transform.block<3,1>(0,3) = position;

        candidate_view_pose.push_back(transform);
    }

    return candidate_view_pose;
}

/**
 * @brief 生成候选视角
 * @return std::vector<Eigen::Matrix4d> 候选视角
*/
inline std::vector<Eigen::Matrix4d> generate_candidate_views(
    std::vector<int> &area_index,
    const Eigen::Vector3d& bbx_unknown_min,
    const Eigen::Vector3d& bbx_unknown_max,
    const double &camera_focal_length,
    const int &random_candidate_points_num,
    const int &longitude_num,
    const double &partition_step_angle_size,
    const double &longitude_step_upper_bound = 90.0,
    const double &longitude_step_lower_bound = 40.0,
    const Eigen::Vector3d &center_bias = Eigen::Vector3d(0,0,0)){
    
    area_index.clear();
    std::vector<Eigen::Matrix4d> candidate_view_pose;

    // 生成一个球体，球心为目标点云的质心，半径为目标点云的半径 + 相机focal_length
    // 计算球体的半径
    double radius = (bbx_unknown_max - bbx_unknown_min).norm() / 2.0 + camera_focal_length;
    // 计算未知bbx的中心
    Eigen::Vector3d center = (bbx_unknown_max + bbx_unknown_min) / 2.0;
    center = center + center_bias;

    std::vector<Eigen::Vector3d> candidate_points;

    // 均匀分布策略是 把纬度分为x份，每个纬度上的点的数量按照对应的经线的长度来分配
    // 生成点的总和为最接近 random_candidate_points_num 的整数

    // 0度对应北极 90度对应赤道 180度对应南极
    // 只采用上半球的[40-90]度 60度的范围
    double longitude_length = 0;
    double start_longitude = M_PI / 180 * longitude_step_lower_bound;
    double end_longitude = M_PI / 180 * longitude_step_upper_bound;
    double latitude_step = (end_longitude - start_longitude) / longitude_num;


    for(int i = 0; i < longitude_num; i++){
        // 计算每个纬度上的经线的总长度
        double latitude = start_longitude + latitude_step * i;
        longitude_length += 2 * M_PI * sin(latitude) * radius;
    }
    // 计算每个点在经线上的间隔
    double longitude_arc = longitude_length / random_candidate_points_num;

    for(int i = 0; i < longitude_num; i++){
        // 维度为 [40，90]只有上半球
        double latitude = start_longitude + latitude_step * i;
        // 计算每个纬度上的经线的总长度
        double local_radius = 2 * M_PI * sin(latitude) * radius;
        // 计算每个点在经线上的数量
        int local_points_num = round(local_radius / longitude_arc);
        for (int j = 0; j < local_points_num; j++){
            double longitude = 2 * M_PI * j / local_points_num;
            Eigen::Vector3d random_point = Eigen::Vector3d(radius * sin(latitude) * cos(longitude),
                                                            radius * sin(latitude) * sin(longitude),
                                                            radius * cos(latitude)) + center;
            // 判断当前longitude角度在哪个区域
            double longitude_angle = longitude / M_PI * 180;
            // 计算当前点所在的区域， 比如当前step为90度， -45~45度区域为 0， 45~135度区域为1 以此类推
            double tmp_longitude_angle = longitude_angle + partition_step_angle_size / 2;
            if(tmp_longitude_angle < 360){
                int area_idx = int(tmp_longitude_angle / partition_step_angle_size);
                area_index.push_back(area_idx);
            }else{
                area_index.push_back(0);
            }

            candidate_points.push_back(random_point);
        }
    }
    
    // 遍历所有的随机点，计算每个点的视角,每个点视角的z轴都指向目标点云的质心
    for(size_t i = 0; i < candidate_points.size(); i++){
        Eigen::Vector3d z_axis = center - candidate_points[i];
        z_axis.normalize();
        // x轴的方向与z轴在水平面垂直
        Eigen::Vector3d x_axis = z_axis.cross(Eigen::Vector3d::UnitZ());
        x_axis.normalize();
        // y轴的方向与z轴和x轴垂直, x,y,z轴构成右手坐标系
        Eigen::Vector3d y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        // 保证z轴方向与x,y轴正交
        z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // 计算旋转矩阵
        Eigen::Matrix3d rotation;
        rotation << x_axis, y_axis, z_axis;

        Eigen::Vector3d position = candidate_points[i];
        
        // 构建齐次变换矩阵
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3,3>(0,0) = rotation;
        transform.block<3,1>(0,3) = position;

        candidate_view_pose.push_back(transform);
    }

    return candidate_view_pose;
}


