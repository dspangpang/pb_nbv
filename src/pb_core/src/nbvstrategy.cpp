#include "../include/pb_core/nbvstrategy.h"

// 使用CGAL库做椭球拟合
#include <CGAL/Cartesian_d.h>
#include <CGAL/MP_Float.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/Approximate_min_ellipsoid_d.h>
#include <CGAL/Approximate_min_ellipsoid_d_traits_d.h>

std::mutex mtx;

nbvstrategy::nbvstrategy(){

    std::string config_file_path = "/root/work_place/pb_nbv/src/pb_core/config/config.json";
        
    // 计算初始的包围盒的位置
    Eigen::Matrix4d object_pose = parseJsonEigenMatrix(config_file_path, "object_pose");
    computeInitBBX(object_pose, bbx_unkonwn_min_, bbx_unkonwn_max_);
    
    // 第一帧视角位姿调整参数
    fist_viewpoint_ues_position_ = parseJsonBool(config_file_path, "fist_viewpoint_ues_position");
    first_viewpoint_distance_factor_ = parseJsonDouble(config_file_path, "first_viewpoint_distance_factor");
    overlooking_angle_ = parseJsonDouble(config_file_path, "first_viewpoint_overlooking_angle");
    Eigen::MatrixXd tmp_posotion = parseJsonEigenMatrix(config_file_path, "first_viewpoint_position");
    first_viewpoint_position_ = tmp_posotion.row(0);

    // 随机生成视角的参数
    random_candidate_points_num_ = parseJsonInt(config_file_path, "random_candidate_points_num");
    longitude_num_ = parseJsonInt(config_file_path, "longitude_num");
    candidate_longitude_angle_ = parseJsonEigenMatrix(config_file_path, "candidate_longitude_angle");
    candidate_center_bias_ = parseJsonEigenMatrix(config_file_path, "candidate_center_bias");
    partition_step_angle_size_ = parseJsonDouble(config_file_path, "partition_step_angle_size");
    // 计算分区的数量
    current_partition_state_.resize(ceil(360.0 / partition_step_angle_size_), 0);

    // 数据保存路径
    save_nbv_tmp_data_ = parseJsonBool(config_file_path, "save_nbv_tmp_data");
    cluster_file_path_ = parseJsonString(config_file_path, "data_root_path") + "cluster/";
    point_projection_file_path_ = parseJsonString(config_file_path, "data_root_path") + "point_projection/";

    // nbv 策略相关参数
    max_gmm_cluster_num_ = parseJsonInt(config_file_path, "max_gmm_cluster_num");
    min_gmm_cluster_num_ = 5;
    // 相机参数解算
    camera_intrinsic_ = parseJsonEigenMatrix(config_file_path, "camera_intrinsic");
    double camera_focal_length_factor = parseJsonDouble(config_file_path, "camera_focal_length_factor");
    analyzeCameraIntrinsic(camera_intrinsic_, camera_focal_length_factor, frustum_points_, camera_focal_length_, image_size_);

    // nbv 计数器
    nbv_cnt_ = 0;

    LOG(INFO) << "nbvstrategy init success";
}
nbvstrategy::~nbvstrategy(){

};

int nbvstrategy::compute_next_view(
    const octomap::ColorOcTree &voxel_map,
    const Eigen::Matrix4d &current_camera_pose, 
    const std::vector<Eigen::Vector3d> &current_frontier_voxels,
    const std::vector<Eigen::Vector3d> &current_occupied_voxels,
    const Eigen::Vector3d &bbx_unknown_min,
    const Eigen::Vector3d &bbx_unknown_max,
    Eigen::Matrix4d& nbv)
{   
    if (nbv_cnt_ == 0){
        LOG (INFO) << "compute_init_view : ";
        if (fist_viewpoint_ues_position_){
            LOG(INFO) << "Use first_viewpoint_position  !";
            nbv = compute_init_view(first_viewpoint_position_);
        }else{
            nbv = compute_init_view();
        }

        current_partition_state_[0] = 1;
        left_index_ = 1;
        right_index_ = current_partition_state_.size() - 1;
        all_scanned_ = false;
    }
    else{

        camera_pose_vec_.push_back(current_camera_pose);
        int best_view_index = 0;
        // 更新bbx_unkonwn_min_和bbx_unkonwn_max_
        bbx_unkonwn_min_ = bbx_unknown_min;
        bbx_unkonwn_max_ = bbx_unknown_max;

        double longitude_step_upper_bound = candidate_longitude_angle_(0, 1);
        double longitude_step_lower_bound = candidate_longitude_angle_(0, 0);
        Eigen::Vector3d center_bias = candidate_center_bias_.row(0);
        std::vector<int> area_index;
        auto views = generate_candidate_views(
            area_index,
            bbx_unkonwn_min_,
            bbx_unkonwn_max_,
            camera_focal_length_,
            random_candidate_points_num_,
            longitude_num_,
            partition_step_angle_size_,
            longitude_step_upper_bound,
            longitude_step_lower_bound,
            center_bias
        );

        LOG (INFO) << "Compute cluster_projection : ";

        std::vector<Eigen::Vector3d> frontier_voxels;
        std::vector<Eigen::Vector3d> occupied_voxels;

        frontier_voxels = current_frontier_voxels;
        occupied_voxels = current_occupied_voxels;

        cluster_ellipsoidization_CGAL(frontier_voxels, occupied_voxels);

        LOG(INFO) << "cluster_ellipsoidization_CGAL compute done! ";
        
        std::vector<double> res(views.size());
        int puttext_pose = int(image_size_.first / 2);

        // 通过全局策略判断下一帧分区
        std::vector<double> candidate_area;
        if (!all_scanned_)
        {
            candidate_area.push_back(left_index_);
            candidate_area.push_back(right_index_);
            LOG(INFO) << "Left_index: " << left_index_;
            LOG(INFO) << "Right_index: " << right_index_;            
        }else{
            LOG(INFO) << "All scanned !";
        }

        // 迭代每个视角
        #pragma omp parallel for 
        for (size_t i = 0; i < views.size(); i++)
        {      
            if (!all_scanned_)
            {   
                // 如果 当前视角不在 candidate_area 中,则直接跳过
                if (std::find(candidate_area.begin(), candidate_area.end(), area_index[i]) == candidate_area.end())
                {
                    res[i] = -std::numeric_limits<double>::max();
                    continue;
                }
            }            
            // views[i] 是相机坐标系到世界坐标系的变换矩阵
            // 计算每个视角的frontier_voxel的面积
            cv::Mat projection_img;

            res[i] = cluster_projection_cv2(
                views[i],
                projection_img);

            if (save_nbv_tmp_data_)
            {
                // 保存投影图
                // 显示 res 的值
                cv::putText(projection_img, "frontier_area: "+std::to_string(res[i]), cv::Point(puttext_pose, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, 8);
                cv::imwrite(point_projection_file_path_ + "projection_img_" + std::to_string(i) + ".png" , projection_img);
            }
        }
        
        best_view_index = nbv_selector(res);
        
        LOG(INFO) << "next_best_view_index: " << best_view_index;
        LOG(INFO) << "next_best_view_res: " << res[best_view_index];
        LOG(INFO) << "next_area_index: " << area_index[best_view_index];
        if (!all_scanned_)
        {
            current_partition_state_[area_index[best_view_index]] = 1;

            if (left_index_ >= right_index_)
            {
                all_scanned_ = true;
            }
            if (area_index[best_view_index] == left_index_)
            {
                left_index_++;
            }else if (area_index[best_view_index] == right_index_)
            {
                right_index_--;
            }
        }
        
        nbv = views[best_view_index];
    }    
    
    nbv_cnt_ ++;

    return 0;
}

std::vector<EllipsoidParam> nbvstrategy::get_ellipsoids(){
    std::vector<EllipsoidParam> ep;        
    ep = ellipsoid_vec_;
    return ep;
}

Eigen::Matrix4d nbvstrategy::compute_init_view(){

    // 计算未知区域的中心位置
    Eigen::Vector3d unknown_center = (bbx_unkonwn_min_ + bbx_unkonwn_max_) / 2.0;

    // 第一帧的视角要位置区域的中心位置,绕世界坐标的y轴顺时针旋转overlooking_angle_角度
    double overlooking_angle_rad = overlooking_angle_ / 180.0 * M_PI;
    Eigen::Vector3d z_axis = Eigen::Vector3d(1,0,0) * cos(overlooking_angle_rad) - Eigen::Vector3d(0,0,1) * sin(overlooking_angle_rad);
    
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

    // position为目标中心点沿着z轴方向向后移动一个目标点云的半径 + 相机focal_length
    // 计算Octmap的包围盒在z_axis轴方向的长度
    double distance = (bbx_unkonwn_max_ - bbx_unkonwn_min_).norm() / 2.0 ;

    Eigen::Vector3d position = 
        unknown_center - z_axis * distance - z_axis * camera_focal_length_ * first_viewpoint_distance_factor_;

    // 构建齐次变换矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = position;

    return transform;

}

Eigen::Matrix4d nbvstrategy::compute_init_view(const Eigen::Vector3d &camera_positon){
    // 构建齐次变换矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    // 相机的z轴方向为 camera_positon 指向 目标中心
    Eigen::Vector3d unknown_center = (bbx_unkonwn_min_ + bbx_unkonwn_max_) / 2.0;
    LOG(INFO) << "unknown_center: " << unknown_center;
    Eigen::Vector3d z_axis = (unknown_center - camera_positon).normalized();

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

    transform.block<3,3>(0,0) = rotation;
    transform.block<3,1>(0,3) = camera_positon;

    return transform;

}


int nbvstrategy::nbv_selector(const std::vector<double>& res)
{

    std::cout << "nbv_selector: ";

    // 从所有view中选择一个最好的
    int best_view_index = -1;
    double best_res = -std::numeric_limits<double>::max();
    for (size_t i = 0; i < res.size(); i++)
    {
        if (res[i] > best_res)
        {
            best_res = res[i];
            best_view_index = i;
        }
    }

    return best_view_index;
}

int nbvstrategy::cluster_ellipsoidization_CGAL(    
    const std::vector<Eigen::Vector3d> &frontier_voxels,
    const std::vector<Eigen::Vector3d> &occupied_voxels){


    typedef CGAL::Cartesian_d<double>                              Kernel;
    typedef CGAL::MP_Float                                         ET;
    typedef CGAL::Approximate_min_ellipsoid_d_traits_d<Kernel, ET> Traits;
    typedef Traits::Point                                          Point;
    typedef std::vector<Point>                                     Point_list;
    typedef CGAL::Approximate_min_ellipsoid_d<Traits>              AME;

    // compute approximation:
    const double eps = 0.01;                // approximation ratio is (1+eps)
    Traits traits;
    const int d = 3;

    ellipsoid_vec_.clear();

    LOG(INFO) << "frontier voxel num: " << frontier_voxels.size();
    LOG(INFO) << "occupied voxel num: " << occupied_voxels.size();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    auto frontier_clusters = gmm_clustering(frontier_voxels);
    auto occupied_clusters = gmm_clustering(occupied_voxels);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    LOG(INFO) << "best_frontier_cluster_num: " << frontier_clusters.size();
    LOG(INFO) << "best_occupied_cluster_num: " << occupied_clusters.size();
    LOG(INFO) << "gmm_clustering execute time: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms";

    if (save_nbv_tmp_data_)
    {
        // 保存聚类结果为pcd文件
        pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        # pragma omp parallel for
        for (size_t i = 0; i < frontier_clusters.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr frontier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t j = 0; j < frontier_clusters[i].size(); j++)
            {
                pcl::PointXYZ p;
                p.x = frontier_clusters[i][j][0];
                p.y = frontier_clusters[i][j][1];
                p.z = frontier_clusters[i][j][2];
                frontier_cloud->push_back(p);
            }
            pcl::io::savePCDFileBinary(cluster_file_path_ + "frontier_cluster_" + std::to_string(i) + ".pcd", *frontier_cloud);
        }
        # pragma omp parallel for
        for (size_t i = 0; i < occupied_clusters.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t j = 0; j < occupied_clusters[i].size(); j++)
            {
                pcl::PointXYZ p;
                p.x = occupied_clusters[i][j][0];
                p.y = occupied_clusters[i][j][1];
                p.z = occupied_clusters[i][j][2];
                occupied_cloud->push_back(p);
            }
            pcl::io::savePCDFileBinary(cluster_file_path_ + "occupied_cluster_" + std::to_string(i) + ".pcd", *occupied_cloud);
        }
    }
    
    // 计算frontier_voxels的最小外接椭球
    double frontier_ellipsoid_num = frontier_clusters.size();
    if (frontier_ellipsoid_num > 0)
    {
        std::vector<EllipsoidParam> frontier_ellipsoid_vec(frontier_clusters.size());
        # pragma omp parallel for
        for (size_t i = 0; i < size_t(frontier_clusters.size()); i++)
        {
            // 把 std::vector<Eigen::Vector3d> 转换为 Eigen::MatrixXd
            Point_list frontier_points;
            for (size_t j = 0; j < frontier_clusters[i].size(); j++)
            {
                std::vector<double> vec(frontier_clusters[i][j].data(), frontier_clusters[i][j].data() + 3);
                frontier_points.push_back(Point(3, vec.begin(), vec.end()));
            }

            AME mel(eps, frontier_points.begin(), frontier_points.end(), traits);

            bool vaild_flag = (mel.is_full_dimensional() && d == 3) ;
            if (!vaild_flag)
            {
                continue;
            }
            
            EllipsoidParam ellipsoid;

            auto radii = mel.axes_lengths_begin();
            auto centroid = mel.center_cartesian_begin();
            auto direction_0 = mel.axis_direction_cartesian_begin(0);
            auto direction_1 = mel.axis_direction_cartesian_begin(1);
            auto direction_2 = mel.axis_direction_cartesian_begin(2);


            ellipsoid.pose = Eigen::Matrix4d::Identity();
            ellipsoid.pose.block<3,1>(0,3) = Eigen::Vector3d(centroid[0], centroid[1], centroid[2]);
            ellipsoid.pose.block<3,3>(0,0) = (Eigen::Matrix3d() << direction_0[0], direction_1[0], direction_2[0],
                                                                    direction_0[1], direction_1[1], direction_2[1],
                                                                    direction_0[2], direction_1[2], direction_2[2]).finished();
            ellipsoid.radii = Eigen::Vector3d(radii[0], radii[1], radii[2]);
            ellipsoid.type = "frontier";

            frontier_ellipsoid_vec[i] = ellipsoid;
        }

        ellipsoid_vec_.insert(ellipsoid_vec_.end(), frontier_ellipsoid_vec.begin(), frontier_ellipsoid_vec.end());

        LOG(INFO) << "frontier ellipsoid compute done !";
    }else
    {
        LOG(INFO) << "frontier ellipsoid empty !";
    }
    
    double occupied_ellipsoid_num = occupied_clusters.size();
    if(occupied_ellipsoid_num > 0){
        // 计算occupied_voxels的最小外接椭球
        std::vector<EllipsoidParam> occupied_ellipsoid_vec(occupied_clusters.size());
        # pragma omp parallel for
        for (size_t i = 0; i < size_t(occupied_clusters.size()); i++)
        {
            // 把 std::vector<Eigen::Vector3d> 转换为 Eigen::MatrixXd
            Point_list occupied_points;
            for (size_t j = 0; j < occupied_clusters[i].size(); j++)
            {
                std::vector<double> vec(occupied_clusters[i][j].data(), occupied_clusters[i][j].data() + 3);
                occupied_points.push_back(Point(3, vec.begin(), vec.end()));
            }

            AME mel(eps, occupied_points.begin(), occupied_points.end(), traits);

            bool vaild_flag = (mel.is_full_dimensional() && d == 3) ;
            if (!vaild_flag)
            {
                continue;
            }
            
            EllipsoidParam ellipsoid;

            auto radii = mel.axes_lengths_begin();
            auto centroid = mel.center_cartesian_begin();
            auto direction_0 = mel.axis_direction_cartesian_begin(0);
            auto direction_1 = mel.axis_direction_cartesian_begin(1);
            auto direction_2 = mel.axis_direction_cartesian_begin(2);
        

            ellipsoid.pose = Eigen::Matrix4d::Identity();
            ellipsoid.pose.block<3,1>(0,3) = Eigen::Vector3d(centroid[0], centroid[1], centroid[2]);
            ellipsoid.pose.block<3,3>(0,0) = (Eigen::Matrix3d() << direction_0[0], direction_1[0], direction_2[0],
                                                                    direction_0[1], direction_1[1], direction_2[1],
                                                                    direction_0[2], direction_1[2], direction_2[2]).finished();
            ellipsoid.radii = Eigen::Vector3d(radii[0], radii[1], radii[2]);
            ellipsoid.type = "occupied";

            occupied_ellipsoid_vec[i] = ellipsoid;
        }
        ellipsoid_vec_.insert(ellipsoid_vec_.end(), occupied_ellipsoid_vec.begin(), occupied_ellipsoid_vec.end());
        LOG(INFO) << "Occupied ellipsoid compute done !";
    }else
    {
        LOG(INFO) << "Occupied ellipsoid empty !";
    }
    
    LOG(INFO) << "ellipsoid_vec_ size: " << ellipsoid_vec_.size();

    return 0;
}

double nbvstrategy::
cluster_projection(
    const Eigen::Matrix4d &camera_pose,
    cv::Mat &projection_img_out){
    
    // 定义一个 3*4 的摄像机矩阵 P
    Eigen::Matrix<double, 3, 4> camera_matrix;
    Eigen::Matrix<double, 3, 4> tmp;
    Eigen::Matrix4d camera_pose_inv = camera_pose.inverse(); // 此处需要的是世界坐标系到相机坐标系的变换
    Eigen::Matrix3d R = camera_pose_inv.block<3,3>(0,0);
    Eigen::Vector3d t = camera_pose_inv.block<3,1>(0,3);
    tmp.block<3,3>(0,0) = R;
    tmp.block<3,1>(0,3) = t;
    camera_matrix = camera_intrinsic_ * tmp;

    // 把 ellipsoid_vec_ 中的中心点坐标换算到相机坐标系下
    std::vector<double> centers_z(ellipsoid_vec_.size());
    # pragma omp parallel for
    for (size_t i = 0; i < ellipsoid_vec_.size(); i++)
    {
        Eigen::Vector3d center;
        center << ellipsoid_vec_[i].pose(0,3), ellipsoid_vec_[i].pose(1,3), ellipsoid_vec_[i].pose(2,3);
        // 正交矩阵的逆等于它的转置
        center = camera_pose.block<3,3>(0,0).transpose() * (center - camera_pose.block<3,1>(0,3));
        centers_z[i] = center[2];
    }

    // 对 centers 按照 z 轴的大小排序
    std::vector<double> weights(centers_z.size());
    std::vector<size_t> idx_vec(centers_z.size());
    std::iota(idx_vec.begin(), idx_vec.end(), 0);
    std::sort(idx_vec.begin(), idx_vec.end(), [&centers_z](size_t i1, size_t i2) {return centers_z[i1] < centers_z[i2];});

    # pragma omp parallel for
    for (size_t i = 0; i < weights.size(); i++)
    {
        weights[i] = 1 * pow(0.5, idx_vec[i]);
    }

    // 定义用于 opencv 使用 ellipse 函数的输入
    std::vector<double> a_vec(ellipsoid_vec_.size());
    std::vector<double> b_vec(ellipsoid_vec_.size());
    std::vector<double> c_vec(ellipsoid_vec_.size());
    std::vector<double> d_vec(ellipsoid_vec_.size());
    std::vector<double> e_vec(ellipsoid_vec_.size());
    std::vector<double> f_vec(ellipsoid_vec_.size());

    // 定义一个图像
    cv::Mat img(cv::Size(image_size_.first, image_size_.second), CV_8UC3);
    img.setTo(cv::Scalar(255, 255, 255));

    // 遍历所有的 ellipsoid 计算投影
    #pragma omp parallel for 
    for (size_t i = 0; i < ellipsoid_vec_.size(); i++)
    {
        auto ellipsoid_matrix_dual = create_ellipsoid_dual_matrix(ellipsoid_vec_[i]);
        if (ellipsoid_matrix_dual == Eigen::Matrix4d::Zero())
        {
            a_vec[i] = 0;
            b_vec[i] = 0;
            c_vec[i] = 0;
            d_vec[i] = 0;
            e_vec[i] = 0;
            f_vec[i] = 0;
            continue;
        }
        auto ellipse_matrix = compute_ellipsoid_projection(camera_matrix, ellipsoid_matrix_dual);
        if (ellipse_matrix == Eigen::Matrix3d::Zero())
        {
            a_vec[i] = 0;
            b_vec[i] = 0;
            c_vec[i] = 0;
            d_vec[i] = 0;
            e_vec[i] = 0;
            f_vec[i] = 0;
            continue;
        }

        // 把椭圆矩阵转换为opencv的椭圆
        a_vec[i] = ellipse_matrix(0, 0);
        b_vec[i] = ellipse_matrix(1, 1);
        c_vec[i] = ellipse_matrix(0, 1) + ellipse_matrix(1, 0);
        d_vec[i] = ellipse_matrix(0, 2) + ellipse_matrix(2, 0);
        e_vec[i] = ellipse_matrix(1, 2) + ellipse_matrix(2, 1);
        f_vec[i] = ellipse_matrix(2, 2);  

    }
    int totalPixels = img.rows * img.cols;

    std::atomic<float> occupied_res(0);
    std::atomic<float> frontier_res(0);
    #pragma omp parallel for
    for (int index = 0; index < totalPixels; index++){
        int k = index / img.cols; // 计算当前像素的行号
        int l = index % img.cols; // 计算当前像素的列号

        double x = l;
        double y = k;
        double tmp_x_x = x * x;
        double tmp_y_y = y * y;
        double tmp_x_y = x * y;

        for (size_t i = 0; i < ellipsoid_vec_.size(); i++){
            if (a_vec[i] == 0 && b_vec[i] == 0 && c_vec[i] == 0 
                && d_vec[i] == 0 && e_vec[i] == 0 && f_vec[i] == 0)
            {
                continue;   
            }
            // 获取当前像素的颜色
            cv::Vec3b color = img.at<cv::Vec3b>(k, l);
            int r = color[2];
            int g = color[1];
            int b = color[0];

            if(ellipsoid_vec_[i].type == "frontier"){
                double value = a_vec[i] * tmp_x_x + b_vec[i] * tmp_y_y + c_vec[i] * tmp_x_y + d_vec[i] * x + e_vec[i] * y + f_vec[i];
                if (value < 0)
                {   
                    if (g == 0)
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(b, 0, 255);
                    }
                    if (g == 255){
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(0, 0, 255);
                    }
                    frontier_res = frontier_res + 255 * weights[i];
                }

            }else if(ellipsoid_vec_[i].type == "occupied"){
                double value = a_vec[i] * tmp_x_x + b_vec[i] * tmp_y_y + c_vec[i] * tmp_x_y + d_vec[i] * x + e_vec[i] * y + f_vec[i];
                if (value < 0)
                {
                    if (g == 0)
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(255, 0, r);
                    }
                    if (g == 255)
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(255, 0, 0);
                    }
                    
                    occupied_res = occupied_res + 255 * weights[i];
                }
            }
        }
    }

    projection_img_out = img.clone();
    double res = frontier_res - occupied_res;
    return res;
}

double nbvstrategy::
cluster_projection_cv2(
    const Eigen::Matrix4d &camera_pose,
    cv::Mat &projection_img_out){
    
    // 定义一个 3*4 的摄像机矩阵 P
    Eigen::Matrix<double, 3, 4> camera_matrix;
    Eigen::Matrix<double, 3, 4> tmp;
    Eigen::Matrix4d camera_pose_inv = camera_pose.inverse(); // 此处需要的是世界坐标系到相机坐标系的变换
    Eigen::Matrix3d R = camera_pose_inv.block<3,3>(0,0);
    Eigen::Vector3d t = camera_pose_inv.block<3,1>(0,3);
    tmp.block<3,3>(0,0) = R;
    tmp.block<3,1>(0,3) = t;
    camera_matrix = camera_intrinsic_ * tmp;

    // 把 ellipsoid_vec_ 中的中心点坐标换算到相机坐标系下
    std::vector<Eigen::Vector3d> centers(ellipsoid_vec_.size());
    # pragma omp parallel for
    for (size_t i = 0; i < ellipsoid_vec_.size(); i++)
    {
        Eigen::Vector3d center;
        center << ellipsoid_vec_[i].pose(0,3), ellipsoid_vec_[i].pose(1,3), ellipsoid_vec_[i].pose(2,3);
        center = camera_pose.block<3,3>(0,0).transpose() * (center - camera_pose.block<3,1>(0,3));
        centers[i] = center;
    }

    // 把 ellipsoid_vec_ 中的中心点坐标换算到相机坐标系下
    std::vector<double> centers_z(ellipsoid_vec_.size());
    # pragma omp parallel for
    for (size_t i = 0; i < ellipsoid_vec_.size(); i++)
    {
        Eigen::Vector3d center;
        center << ellipsoid_vec_[i].pose(0,3), ellipsoid_vec_[i].pose(1,3), ellipsoid_vec_[i].pose(2,3);
        center = camera_pose.block<3,3>(0,0).transpose() * (center - camera_pose.block<3,1>(0,3));
        centers_z[i] = center[2];
    }

    // 对 centers 按照 z 轴的大小排序
    std::vector<double> weights(centers_z.size());
    std::vector<size_t> idx_vec(centers_z.size());
    std::iota(idx_vec.begin(), idx_vec.end(), 0);
    std::sort(idx_vec.begin(), idx_vec.end(), [&centers_z](size_t i1, size_t i2) {return centers_z[i1] < centers_z[i2];});

    # pragma omp parallel for
    for (size_t i = 0; i < weights.size(); i++)
    {
        weights[i] = 1 * pow(0.5, idx_vec[i]);
    }

    // 遍历所有的 ellipsoid 计算投影
    std::vector<cv::Mat> img_vec(ellipsoid_vec_.size());
    #pragma omp parallel for 
    for (size_t i = 0; i < ellipsoid_vec_.size(); i++)
    {
        img_vec[i] = cv::Mat(cv::Size(image_size_.first, image_size_.second), CV_8UC1);
        img_vec[i].setTo(cv::Scalar(0));
        auto ellipsoid_matrix_dual = create_ellipsoid_dual_matrix(ellipsoid_vec_[i]);
        if (ellipsoid_matrix_dual == Eigen::Matrix4d::Zero())
        {
            continue;
        }
        auto ellipse_matrix = compute_ellipsoid_projection(camera_matrix, ellipsoid_matrix_dual);
        if (ellipse_matrix == Eigen::Matrix3d::Zero())
        {
            continue;
        }

        // 把椭圆矩阵转换为opencv的椭圆
        // A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
        double A = ellipse_matrix(0, 0);
        double B = ellipse_matrix(0, 1) + ellipse_matrix(1, 0);
        double C = ellipse_matrix(1, 1);
        double D = ellipse_matrix(0, 2) + ellipse_matrix(2, 0);
        double E = ellipse_matrix(1, 2) + ellipse_matrix(2, 1);
        double F = ellipse_matrix(2, 2);
        
        // 计算椭圆的中心
        double x0 = (2*C*D - B*E) / (B*B - 4*A*C);
        double y0 = (2*A*E - B*D) / (B*B - 4*A*C);

        // 计算旋转角度
        double theta = -0.5 * atan2(B, C - A) * 180 / CV_PI; // 转换为度

        // 计算长轴和短轴
        double a_length = sqrt((A*x0*x0 + B*x0*y0 + C*y0*y0 - F)/(A*cos(theta)*cos(theta) + B*sin(theta)*cos(theta) + C*sin(theta)*sin(theta)));
        double b_length = sqrt((A*x0*x0 + B*x0*y0 + C*y0*y0 - F)/(C*cos(theta)*cos(theta) - B*sin(theta)*cos(theta) + A*sin(theta)*sin(theta)));
        
        if (a_length < 0 || b_length < 0)
        {
            continue;
        }
        
        try{
            // cv::ellipse(img_vec[i], cv::Point(x0, y0), cv::Size(a_length, b_length), theta, 0, 360, 255, -1);
            cv::ellipse(img_vec[i], cv::Point(int(x0), int(y0)), cv::Size(int(a_length), int(b_length)), theta, 0, 360, 255, -1); 
        }catch (const cv::Exception& e) {
            // std::cerr << "Caught cv::Exception: " << e.what() << std::endl;
        }
    }

    // 把所有的投影图像叠加
    std::atomic<float> occupied_res(0);
    std::atomic<float> frontier_res(0);
    cv::Mat img(cv::Size(image_size_.first, image_size_.second), CV_8UC3);
    img.setTo(cv::Scalar(255, 255, 255));
    int totalPixels = img.rows * img.cols;
    try{
        #pragma omp parallel for 
        for (int index = 0; index < totalPixels; index++){
            int k = index / img.cols; // 计算当前像素的行号
            int l = index % img.cols; // 计算当前像素的列号
            for (size_t i = 0; i < ellipsoid_vec_.size(); i++){
                cv::Vec3b color_img = img.at<cv::Vec3b>(k, l);
                int g_img = color_img[1];
                uchar value = img_vec[i].at<uchar>(k, l);
                if (value != 255)
                {
                    continue;
                }
                if (ellipsoid_vec_[i].type == "occupied")
                {
                    occupied_res = occupied_res + value * weights[i];
                    if (g_img == 255)
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(255, 0, 0);
                    }else
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(255, 0, 255);
                    }
                }else if (ellipsoid_vec_[i].type == "frontier")
                {
                    frontier_res = frontier_res + value * weights[i];
                    if (g_img == 255)
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(0, 0, 255);
                    }else
                    {
                        img.at<cv::Vec3b>(k, l) = cv::Vec3b(255, 0, 255);
                    }
                }
            }
        }
    } catch (const cv::Exception& e) {
        // std::cerr << "Caught cv::Exception: " << e.what() << std::endl;
    }

    double res = frontier_res - occupied_res;
    projection_img_out = img.clone();

    return res;
}

bool nbvstrategy::computeRayBoxIntersection(
    const Eigen::Vector3d& ray_origin, 
    const Eigen::Vector3d& ray_direction, 
    const Eigen::Vector3d& box_min, 
    const Eigen::Vector3d& box_max, 
    Eigen::Vector3d& intersection){
    
    double tmin = (box_min.x() - ray_origin.x()) / ray_direction.x();
    double tmax = (box_max.x() - ray_origin.x()) / ray_direction.x();

    if (tmin > tmax) std::swap(tmin, tmax);

    double tymin = (box_min.y() - ray_origin.y()) / ray_direction.y();
    double tymax = (box_max.y() - ray_origin.y()) / ray_direction.y();

    if (tymin > tymax) std::swap(tymin, tymax);

    if ((tmin > tymax) || (tymin > tmax))
        return false;

    if (tymin > tmin)
        tmin = tymin;

    if (tymax < tmax)
        tmax = tymax;

    double tzmin = (box_min.z() - ray_origin.z()) / ray_direction.z();
    double tzmax = (box_max.z() - ray_origin.z()) / ray_direction.z();

    if (tzmin > tzmax) std::swap(tzmin, tzmax);

    if ((tmin > tzmax) || (tzmin > tmax))
        return false;

    if (tzmin > tmin)
        tmin = tzmin;

    if (tzmax < tmax)
        tmax = tzmax;

    intersection = ray_origin + ray_direction * tmin;
    return true;
}

std::vector<std::vector<Eigen::Vector3d>> nbvstrategy::
dbscan_clustering(const std::vector<Eigen::Vector3d> &voxels){

    std::vector<std::vector<Eigen::Vector3d>> clustered_clouds;

    // 使用 pcl dbscann 进行聚类
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& voxel : voxels) {
        cloud->points.push_back(pcl::PointXYZ(voxel.x(), voxel.y(), voxel.z()));
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    // 创建KdTree对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 创建EuclideanClusterExtraction对象
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01); // 设置近邻搜索的搜索半径为2cm
    ec.setMinClusterSize(10); // 
    ec.setMaxClusterSize(25000); // 设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod(tree); // 设置点云的搜索机制
    ec.setInputCloud(cloud);

    // 执行聚类，将结果存储到一个std::vector中
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 遍历聚类结果,保存进clustered_clouds
    for (const auto& indices : cluster_indices) {
        std::vector<Eigen::Vector3d> cluster;
        for (const auto& index : indices.indices) {
            cluster.push_back(Eigen::Vector3d(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z));
        }
        clustered_clouds.push_back(cluster);
    }

    return clustered_clouds;
}

std::vector<std::vector<Eigen::Vector3d>> nbvstrategy::
gmm_clustering(const std::vector<Eigen::Vector3d> &voxels){

    // 创建一个数据集
    cv::Mat samples(voxels.size(), 3, CV_32FC1);
    for (size_t i = 0; i < voxels.size(); i++)
    {
        samples.at<float>(i, 0) = voxels[i][0];
        samples.at<float>(i, 1) = voxels[i][1];
        samples.at<float>(i, 2) = voxels[i][2];
    }
    if (samples.empty())
    {
        std::vector<std::vector<Eigen::Vector3d>> zero_clouds;
        return zero_clouds;
    }
    
    std::vector<std::vector<Eigen::Vector3d>> clustered_clouds;

    if (samples.rows >= 2*max_gmm_cluster_num_)
    {
        // 创建并训练 GMM
        cv::Mat output_labels;
        int gmm_cnt = max_gmm_cluster_num_ - min_gmm_cluster_num_ + 1;
        // 用于多线程存储
        std::vector<double> cluster_values(gmm_cnt);
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> clustered_clouds_vec(gmm_cnt);

        #pragma omp parallel for
        for (size_t i = min_gmm_cluster_num_; i <= size_t(max_gmm_cluster_num_); i++)
        {   
            if (size_t(samples.rows) < i)
            {
                cluster_values[i] = DBL_MIN;
                continue;
            }
            
            cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
            em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
            em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
            em_model->setClustersNumber(i); // 高斯混合模型的数量
            em_model->trainEM(samples);

            // 使用训练好的 GMM 进行预测
            cv::Mat labels;
            cv::Mat logLikelihoods;
            em_model->predict(samples, labels);
            double total_log_likelihood = 0.0;
            // Calculate likelihood
            std::vector<std::vector<Eigen::Vector3d>> clustered_clouds_tmp(i);
            // 提取每个cluster的点云
            for (size_t j = 0; j < size_t(samples.rows); j++)
            {
                double max = -1;
                int max_index = -1;

                // 找到最大概率及其索引
                for (size_t k = 0; k < i; k++)
                {
                    double current_prob = labels.row(j).at<double>(k);
                    if(current_prob > max){
                        max = current_prob;
                        max_index = k;
                    }
                }

                // 假设max是概率，计算对数似然
                if (max > 0) {
                    double log_likelihood = log(max); // 使用对数函数计算对数似然
                    total_log_likelihood += log_likelihood; // 累加到总对数似然
                }

                clustered_clouds_tmp[max_index].push_back(voxels[j]);
            }

            // 计算评价函数
            // 评价函数为 total_log_likelihood 最大似然函数的的值 - 每个cluster 中的点的数量的倒数 * 2
            double value = total_log_likelihood;
            // bic 准则
            value = 3*log(samples.rows) - 2 * value; 
            // for (size_t j = 0; j < i; j++)
            // {
            //     if (clustered_clouds_tmp[j].size() > 1)
            //     {
            //         value -= 1.0 * 1.0 / clustered_clouds_tmp[j].size();
            //     }
            //     else{
            //         value -= 1.0;
            //     }
            // }
            // mtx.lock();
            // LOG(INFO) << "cluster num: " << i << " value: " << value;
            // mtx.unlock();
            cluster_values[i-min_gmm_cluster_num_] = value;
            clustered_clouds_vec[i-min_gmm_cluster_num_] = clustered_clouds_tmp;
        }

        // // 在 cluster_values 中找出最大值的索引
        // int max_idx = std::distance(cluster_values.begin(), std::max_element(cluster_values.begin(), cluster_values.end()));
        // clustered_clouds = clustered_clouds_vec[max_idx];
        // 在 cluster_values 中找出最小值的索引
        int min_idx = std::distance(cluster_values.begin(), std::min_element(cluster_values.begin(), cluster_values.end()));
        clustered_clouds = clustered_clouds_vec[min_idx];
    }
    else if (samples.rows > 3)
    {
        int cluster_num = samples.rows/2;
        cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
        em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_SPHERICAL);
        em_model->setClustersNumber(cluster_num); // 高斯混合模型的数量
        clustered_clouds.resize(cluster_num);
        em_model->trainEM(samples);
        // 使用训练好的 GMM 进行预测
        cv::Mat labels;
        em_model->predict(samples, labels);

        // 提取每个cluster的点云
        for (size_t j = 0; j < size_t(samples.rows); j++)
        {
            double max = -1;
            int max_index = -1;

            // 找到最大概率及其索引
            for (size_t k = 0; k < size_t(cluster_num); k++)
            {
                double current_prob = labels.row(j).at<double>(k);
                if(current_prob > max){
                    max = current_prob;
                    max_index = k;
                }
            }

            clustered_clouds[max_index].push_back(voxels[j]);
        }
    }else{
        clustered_clouds.resize(1);
        for (size_t i = 0; i < size_t (samples.rows); i++)
        {
            clustered_clouds[0].push_back(voxels[i]);
        }
    }
    

    return clustered_clouds;
}

octomap::point3d nbvstrategy::
to_oct3d(const Eigen::Vector3d& v)
{
    return octomap::point3d(v.x(), v.y(), v.z());
}

Eigen::Vector3d nbvstrategy::
to_eigen3d(const octomap::point3d& p)
{
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

Eigen::Matrix4d nbvstrategy::
create_ellipsoid_matrix(const EllipsoidParam &param){
    
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero(); 
    Eigen::Vector3d radii_pow = param.radii.array().square(); 
    Eigen::Vector3d radii_inv = radii_pow.array().inverse();
    Eigen::Matrix4d transformation = param.pose; 

    // 将radii_inv设置为matrix的前3x3的对角矩阵
    matrix.block<3,3>(0,0) = radii_inv.asDiagonal();
    matrix(3, 3) = -1;

    matrix = transformation * matrix * transformation.transpose(); // 计算最终的矩阵

    return matrix;
}

Eigen::Matrix4d nbvstrategy::
create_ellipsoid_dual_matrix(const EllipsoidParam &param){
    
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero(); 
    Eigen::Vector3d radii_pow = param.radii.array().square(); 
    Eigen::Vector3d radii_inv = radii_pow.array().inverse();
    Eigen::Matrix4d transformation = param.pose; 

    // 将radii_inv设置为matrix的前3x3的对角矩阵
    matrix.block<3,3>(0,0) = radii_inv.asDiagonal();
    matrix(3, 3) = -1;

    double det = matrix.determinant(); 
    if (det == 0) {
        std::cout << "The determinant of the matrix is 0, the matrix is not invertible." << std::endl;
        return Eigen::Matrix4d::Zero(); 
    } else {
        Eigen::Matrix4d matrix_dual_origin = matrix.inverse(); 
        // std::cout << "Matrix Dual Origin:\n" << matrix_dual_origin << std::endl;
        Eigen::Matrix4d matrix_dual = transformation * matrix_dual_origin * transformation.transpose(); // 计算最终的矩阵
        // std::cout << "Matrix Dual:\n" << matrix_dual << std::endl;
        return matrix_dual;
    }
}

Eigen::Matrix3d nbvstrategy::
compute_ellipsoid_projection(
    const Eigen::Matrix<double, 3, 4> camera_matrix,
    const Eigen::Matrix4d ellipsoid_matrix_dual){
    

    Eigen::Matrix3d ellipse_dual = camera_matrix * 
                                    ellipsoid_matrix_dual * 
                                    camera_matrix.transpose();

    double det = ellipse_dual.determinant(); 
    if (det == 0) {
        std::cout << "The determinant of the matrix is 0, the matrix is not invertible." << std::endl;
        return Eigen::Matrix3d::Zero(); 
    } else {
        Eigen::Matrix3d ellipse = ellipse_dual.inverse();
        return ellipse;
    }

}


