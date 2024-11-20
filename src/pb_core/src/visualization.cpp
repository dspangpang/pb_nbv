#include "../include/pb_core/visualization.h"

visualization::visualization():viz_("viz_visualization"){

    std::string config_file_path = "/root/work_place/pb_nbv/src/pb_core/config/config.json";

    set_close_flag(true);
    set_update_flag(false);

    // 相机参数解算
    Eigen::MatrixXd camera_intrinsic = parseJsonEigenMatrix(config_file_path, "camera_intrinsic");
    double camera_focal_length_factor = parseJsonDouble(config_file_path, "camera_focal_length_factor");
    std::pair<int, int> image_size;
    analyzeCameraIntrinsic(camera_intrinsic, camera_focal_length_factor, frustum_points_, camera_focal_length_, image_size);

    random_candidate_points_num_ = parseJsonInt(config_file_path, "random_candidate_points_num");
    longitude_num_ = parseJsonInt(config_file_path, "longitude_num");
    candidate_longitude_angle_ = parseJsonEigenMatrix(config_file_path, "candidate_longitude_angle");
    candidate_center_bias_ = parseJsonEigenMatrix(config_file_path, "candidate_center_bias");
    // 可视化参数设置
    show_frustum_ = parseJsonBool(config_file_path, "show_frustum");
    show_voxel_map_ = parseJsonBool(config_file_path, "show_voxel_map");
    show_ellipsoid_ = parseJsonBool(config_file_path, "show_ellipsoid");
    show_candidate_points_ = parseJsonBool(config_file_path, "show_candidate_points");
    show_candidate_frames_ = parseJsonBool(config_file_path, "show_candidate_frames");
    show_bbx_ = parseJsonBool(config_file_path, "show_bbx");
    show_best_view_frames_ = parseJsonBool(config_file_path, "show_best_view_frames");

    // cpp 全变量初始化
    current_view_cnt_ = 0;
    total_view_cnt_ = random_candidate_points_num_;

    LOG(INFO) << "visualization init success.";
}

visualization::~visualization(){
    set_close_flag(true);
}

void visualization::catpure_frame(
    const Eigen::Matrix4d &camera_pose, 
    const octomap::ColorOcTree &voxel_map_tree,
    const Eigen::Vector3d &bbx_unknown_min,
    const Eigen::Vector3d &bbx_unknown_max){

    camera_pose_.push_back(camera_pose);
    voxel_map_.clear();
    for (auto it = voxel_map_tree.begin_leafs(); it != voxel_map_tree.end_leafs(); ++it)
    {
        Eigen::Vector3d point(it.getX(), it.getY(), it.getZ());
        Eigen::Vector3i color(it->getColor().r, it->getColor().g, it->getColor().b);
        // 跳过显示empty体素
        if (color == Eigen::Vector3i(255, 255, 255))
        {
            continue;
        }
        // 跳过未扫描到的体素
        if (color == Eigen::Vector3i(0, 0, 0))
        {
            continue;
        }

        voxel_map_.push_back(std::make_pair(point, color));

        voxel_resolution_ = it.getSize();
    }

    // 输出体素地图的个数
    LOG(INFO) << "visualization voxel_map size: " << voxel_map_.size();

    bbx_unknown_min_ = bbx_unknown_min;
    bbx_unknown_max_ = bbx_unknown_max;

    set_update_flag(true);
}

void visualization::catpure_frame(
    const Eigen::Matrix4d &camera_pose,
    const std::vector<EllipsoidParam> ellipsoid_vec,
    const octomap::ColorOcTree &voxel_map_tree,
    const Eigen::Vector3d &bbx_unknown_min,
    const Eigen::Vector3d &bbx_unknown_max)
{
    ellipsoid_vec_ = ellipsoid_vec;
    catpure_frame(camera_pose, voxel_map_tree, bbx_unknown_min, bbx_unknown_max);
}
 
void visualization::set_update_flag(const bool &update_flag)
{
    my_mutex_.lock();
    update_flag_ = update_flag;
    my_mutex_.unlock();
}

bool visualization::get_update_flag()
{
    bool update_flag;
    update_flag = this->update_flag_;
    
    return update_flag;
}

void visualization::set_close_flag(const bool &close_flag)
{
    my_mutex_.lock();
    window_should_close_ = close_flag;
    my_mutex_.unlock();
}

bool visualization::get_close_flag()
{
    bool close_flag;
    close_flag = this->window_should_close_;
    
    return close_flag;
}

void visualization::visualize_result(){

    set_close_flag(false);
    set_update_flag(false);

    // 创建一个opencv viz 可视化窗口
    // cv::viz::Viz3d viz("viz_visualization");
    // 设置背景为黑色
    viz_.setBackgroundColor(cv::viz::Color::white());
    // 注册键盘回调函数set_close_flag
    viz_.registerKeyboardCallback([](const cv::viz::KeyboardEvent &event, void* t){
        if (event.action == cv::viz::KeyboardEvent::KEY_DOWN)
        {
            if (event.code == 27)
            {
                static_cast<visualization*>(t)->set_close_flag(true);
            }
        }

        // 如果按下空格键，更新viz显示的观测视角
        if (event.action == cv::viz::KeyboardEvent::KEY_DOWN)
        {
            if (event.code == 32)
            {
                static_cast<visualization*>(t)->update_viz_views();
            }
        }

    }, this);

    std::cout << "press esc to exit." << std::endl;

    // 判断vis 是否要关闭
    while (!get_close_flag())
    {
        if (get_update_flag())
        {
            set_update_flag(false);

            // 画出世界坐标系和物体大致的bbx

            // 清除屏幕
            viz_.removeAllWidgets();
            // 显示原点坐标系
            viz_.showWidget("world", cv::viz::WCoordinateSystem(0.2));
            if (show_bbx_)
            {
                // 画一个个lineset 包围unknown的区域
                auto unknown_bbx = get_unknown_bbx();
                for (size_t i = 0; i < unknown_bbx.size(); i++)
                {
                    viz_.showWidget("unknown_bbx_" + std::to_string(i), unknown_bbx[i]);
                }
                LOG(INFO) << "update unknown bbx done!"; 
            }

            // 在这里处理新的数据，例如更新立方体的位置或者大小
            if(show_voxel_map_){
                for (size_t i = 0; i < voxel_map_.size(); i++)
                {
                    double side = voxel_resolution_ / 2;
                    // 创建一个立方体
                    cv::viz::WCube cube(cv::Point3f(voxel_map_[i].first[0]-side, voxel_map_[i].first[1]-side, voxel_map_[i].first[2]-side), 
                                        cv::Point3f(voxel_map_[i].first[0]+side, voxel_map_[i].first[1]+side, voxel_map_[i].first[2]+side), 
                                        false, cv::viz::Color(voxel_map_[i].second[2], voxel_map_[i].second[1], voxel_map_[i].second[0]));
                    // 显示立方体
                    viz_.showWidget("voxel_" + std::to_string(i), cube);
                }
                LOG(INFO) << "update voxel map done!"; 
            }

            if (show_best_view_frames_)
            {
                for (size_t cnt = 0; cnt < camera_pose_.size(); ++cnt)
                {
                    double frame_size = 0;
                    if(cnt == camera_pose_.size() -1 ){
                        frame_size = 0.3;
                    }else{
                        frame_size = 0.2;
                    }
                    // 在OpenCV中，我们可以直接创建一个坐标系c
                    cv::viz::WCoordinateSystem coord(frame_size);
                    // 把坐标系变换到相机的位姿
                    // 把camera_pose_[cnt] 转换成cv::Affine3d
                    cv::Mat cv_camera_pose_mat;
                    cv::eigen2cv(camera_pose_[cnt], cv_camera_pose_mat);
                    cv::Affine3d cv_camera_pose(cv_camera_pose_mat);
                    coord.applyTransform(cv_camera_pose);
                    // 显示坐标系
                    viz_.showWidget("frustum_coord_" + std::to_string(cnt), coord);
                    if (show_frustum_)
                    {
                        // 创建一个视锥
                        auto frustum = get_frustum(1.0, camera_pose_[cnt]);
                        // 显示视锥
                        for (size_t i = 0; i < frustum.size(); i++)
                        {
                            viz_.showWidget("frustum_" + std::to_string(cnt) + "_" + std::to_string(i), frustum[i]);
                        }
                    }
                }
                LOG(INFO) << "update best views frames done!: "; 
            }
            
            if (show_ellipsoid_)
            {
                if (!ellipsoid_vec_.empty())
                {
                    for (size_t i = 0; i < ellipsoid_vec_.size(); i++)
                    {
                        // 创建一个mesh球
                        cv::viz::WSphere sphere(cv::Point3d(0, 0, 0), 1.0, 10, cv::viz::Color::red());
                        // 创建缩放矩阵
                        Eigen::Matrix4d scaleMatrix = Eigen::Matrix4d::Identity();
                        scaleMatrix(0, 0) = ellipsoid_vec_[i].radii[0];
                        scaleMatrix(1, 1) = ellipsoid_vec_[i].radii[1];
                        scaleMatrix(2, 2) = ellipsoid_vec_[i].radii[2];
                        // 将Eigen矩阵转换为OpenCV矩阵
                        cv::Mat cv_scaleMatrix;
                        cv::eigen2cv(scaleMatrix, cv_scaleMatrix);
                        cv::Affine3d scale(cv_scaleMatrix);
                        sphere.applyTransform(scale);
                        // 将Eigen矩阵转换为OpenCV矩阵
                        cv::Mat cv_transform;
                        cv::eigen2cv(ellipsoid_vec_[i].pose, cv_transform);
                        cv::Affine3d pose(cv_transform);
                        sphere.applyTransform(pose);
                        // 设置椭球的颜色
                        sphere.setColor(cv::viz::Color::blue());
                        if(ellipsoid_vec_[i].type == "frontier"){
                            sphere.setColor(cv::viz::Color::red());
                        }
                        else if(ellipsoid_vec_[i].type == "occupied"){
                            sphere.setColor(cv::viz::Color::blue());
                        }

                        sphere.setRenderingProperty(cv::viz::SHADING, cv::viz::SHADING_PHONG);
                        sphere.setRenderingProperty(cv::viz::AMBIENT, 1);

                        // 显示椭球
                        viz_.showWidget("ellipsoid_" + std::to_string(i), sphere);
                    }

                    LOG(INFO) << "update ellipsoid done!"; 
                }else
                {
                    LOG(ERROR) << "ellipsoid_vec_ is empty! "; 
                }
                
            }

            if(show_candidate_points_ || show_candidate_frames_){

                double longitude_step_upper_bound = candidate_longitude_angle_(0, 1);
                double longitude_step_lower_bound = candidate_longitude_angle_(0, 0);
                Eigen::Vector3d center_bias = candidate_center_bias_.row(0);
                candidate_points_ = generate_candidate_views(
                    bbx_unknown_min_,
                    bbx_unknown_max_,
                    camera_focal_length_,
                    random_candidate_points_num_,
                    longitude_num_,
                    longitude_step_upper_bound,
                    longitude_step_lower_bound,
                    center_bias
                    );
            }

            if (show_candidate_frames_)
            {
                for (size_t i = 0; i < candidate_points_.size(); i++)
                {
                    // 在OpenCV中，我们可以直接创建一个坐标系
                    cv::viz::WCoordinateSystem coord(0.1);
                    // 把坐标系变换到相机的位姿
                    // 把camera_pose_[cnt] 转换成cv::Affine3d
                    cv::Mat cv_camera_pose_mat;
                    cv::eigen2cv(candidate_points_[i], cv_camera_pose_mat);
                    cv::Affine3d cv_camera_pose(cv_camera_pose_mat);
                    coord.applyTransform(cv_camera_pose);
                    // 显示坐标系
                    viz_.showWidget("candidate_coord_" + std::to_string(i), coord);
                    // 添加文字 并设置文字的位姿为cv_camera_pose
                    // 创建一个沿Y轴翻转的矩阵
                    cv::Mat flip = (cv::Mat_<double>(4,4) << 
                        1, 0, 0, 0,
                        0, -1, 0, 0,
                        0, 0, -1, 0,
                        0, 0, 0, 1);
                    cv::Affine3d cv_txt_pose(cv_camera_pose_mat*flip);
                    viz_.showWidget("candidate_text_" + std::to_string(i), 
                                    cv::viz::WText3D(std::to_string(i), 
                                    cv::Point3d(0.1, 0, 0), 0.01, 
                                    true, cv::viz::Color::green()), 
                                    cv_txt_pose);
                    
                }

                LOG(INFO) << "update candidate frames done!";
            }

        }
        // 更新视图并处理事件
        viz_.spinOnce(1, true);
    }
}

std::vector<cv::viz::WLine> visualization::
    get_unknown_bbx()
{

    // 把bbx_unknown_min_ bbx_unknown_max_ 转成cv::point3d
    // 作为正方体的8个定点
    cv::Point3d cv_p1(bbx_unknown_min_[0], bbx_unknown_min_[1], bbx_unknown_min_[2]);
    cv::Point3d cv_p2(bbx_unknown_min_[0], bbx_unknown_min_[1], bbx_unknown_max_[2]);
    cv::Point3d cv_p3(bbx_unknown_min_[0], bbx_unknown_max_[1], bbx_unknown_min_[2]);
    cv::Point3d cv_p4(bbx_unknown_min_[0], bbx_unknown_max_[1], bbx_unknown_max_[2]);
    cv::Point3d cv_p5(bbx_unknown_max_[0], bbx_unknown_min_[1], bbx_unknown_min_[2]);
    cv::Point3d cv_p6(bbx_unknown_max_[0], bbx_unknown_min_[1], bbx_unknown_max_[2]);
    cv::Point3d cv_p7(bbx_unknown_max_[0], bbx_unknown_max_[1], bbx_unknown_min_[2]);
    cv::Point3d cv_p8(bbx_unknown_max_[0], bbx_unknown_max_[1], bbx_unknown_max_[2]);

    // 画出正方体的12条边
    std::vector<cv::viz::WLine> unknown_bbx;
    unknown_bbx.push_back(cv::viz::WLine(cv_p1, cv_p2, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p1, cv_p3, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p1, cv_p5, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p2, cv_p4, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p2, cv_p6, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p3, cv_p4, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p3, cv_p7, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p4, cv_p8, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p5, cv_p6, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p5, cv_p7, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p6, cv_p8, cv::viz::Color::green()));
    unknown_bbx.push_back(cv::viz::WLine(cv_p7, cv_p8, cv::viz::Color::green()));
    
    return unknown_bbx;
}

std::vector<cv::viz::WLine> visualization::
get_frustum(const double depth, const Eigen::Matrix4d frustum_pose){
    // 计算视锥底面的四个顶点
    Eigen::Vector3d p1 = frustum_points_[0] * depth;
    Eigen::Vector3d p2 = frustum_points_[1] * depth;
    Eigen::Vector3d p3 = frustum_points_[2] * depth;
    Eigen::Vector3d p4 = frustum_points_[3] * depth;

    // 把视锥的顶点转换到世界坐标系下
    p1 = frustum_pose.block<3,3>(0,0) * p1 + frustum_pose.block<3,1>(0,3);
    p2 = frustum_pose.block<3,3>(0,0) * p2 + frustum_pose.block<3,1>(0,3);
    p3 = frustum_pose.block<3,3>(0,0) * p3 + frustum_pose.block<3,1>(0,3);
    p4 = frustum_pose.block<3,3>(0,0) * p4 + frustum_pose.block<3,1>(0,3);

    // 把p1 p2 p3 p4 转成cv::point3d
    cv::Point3d cv_p1(p1[0], p1[1], p1[2]);
    cv::Point3d cv_p2(p2[0], p2[1], p2[2]);
    cv::Point3d cv_p3(p3[0], p3[1], p3[2]);
    cv::Point3d cv_p4(p4[0], p4[1], p4[2]);

    // 设置零点
    Eigen::Vector3d zero = frustum_pose.block<3,1>(0,3);
    cv::Point3d cv_zero(zero[0], zero[1], zero[2]);

    // 创建一个表示相机视锥的线段集合
    std::vector<cv::viz::WLine> frustum;
    
    frustum.push_back(cv::viz::WLine(cv_p1, cv_p2, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p2, cv_p3, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p3, cv_p4, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p4, cv_p1, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p1, cv_zero, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p2, cv_zero, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p3, cv_zero, cv::viz::Color::red()));
    frustum.push_back(cv::viz::WLine(cv_p4, cv_zero, cv::viz::Color::red()));

    return frustum;
}

void visualization::update_viz_views(){

    // 修改视角
    if (show_candidate_points_ && !candidate_points_.empty())
    {
        current_view_cnt_ = (current_view_cnt_ + 1) % total_view_cnt_;
        Eigen::Matrix4d camera_pose = candidate_points_[current_view_cnt_-1];

        cv::Mat cv_camera_pose_mat;
        cv::eigen2cv(camera_pose, cv_camera_pose_mat);
        cv::Affine3d cv_camera_pose(cv_camera_pose_mat);

        // 设置新的相机位姿
        viz_.setViewerPose(cv_camera_pose);
    }

}
