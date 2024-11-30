#include "../include/pb_core/voxelstruct.h"
#include <chrono>
// 全局维护的 _voxel_map
// 用于判断voxel_map中的点是否被扫描到
octomap::ColorOcTree _voxel_map(1);
// 用于判断当前的voxel_map中的点是否被扫描到
octomap::ColorOcTree _bbx_voxel_map(1);

voxelstruct::voxelstruct(){

    std::string config_file_path = "/root/work_place/pb_nbv/src/pb_core/config/config.json";

    occupied_voxels_.clear();
    voxel_resolution_ = parseJsonDouble(config_file_path, "voxel_resolution");
    _voxel_map.setResolution(voxel_resolution_);
    _bbx_voxel_map.setResolution(voxel_resolution_);
    _voxel_map.clear();
    _bbx_voxel_map.clear();

    // 相机参数解算
    Eigen::MatrixXd camera_intrinsic = parseJsonEigenMatrix(config_file_path, "camera_intrinsic");
    double camera_focal_length_factor = parseJsonDouble(config_file_path, "camera_focal_length_factor");
    std::pair<int, int> image_size;
    analyzeCameraIntrinsic(camera_intrinsic, camera_focal_length_factor, frustum_points_, camera_focal_length_, image_size);

    ray_trace_step_ = parseJsonDouble(config_file_path, "ray_trace_step");
    surrounding_voxels_radius_ = parseJsonInt(config_file_path, "surrounding_voxels_radius");

    LOG(INFO) << "voxelstruct init success";
}
voxelstruct::~voxelstruct() = default;


octomap::ColorOcTree voxelstruct::
update_voxel_map(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const Eigen::Matrix4d &camera_pose,
    std::vector<Eigen::Vector3d> &output_frontier_voxels,
    std::vector<Eigen::Vector3d> &output_occupied_voxels,
    Eigen::Vector3d &bbx_unknown_min,
    Eigen::Vector3d &bbx_unknown_max){

    output_frontier_voxels.clear();
    output_occupied_voxels.clear();

    std::vector<Eigen::Vector3d> frontier_voxels;
    
    octomap::ColorOcTree voxel_map(voxel_resolution_);

    bool is_first = true;
    if (occupied_voxels_.size() > 0)
    {
        is_first = false;
    }

    if (cloud->size() == 0)
    {
        LOG(WARNING) << "cloud is empty";
        return voxel_map;
    }
    
#ifdef DEBUG
    // 检查一下当前_voxel_map 和 _bbx_voxel_map 的大小
    LOG(ERROR) << "current _voxel_map getNumLeafNodes: " << _voxel_map.getNumLeafNodes();
    LOG(ERROR) << "current _bbx_voxel_map getNumLeafNodes: " << _bbx_voxel_map.getNumLeafNodes();
#endif
    

    // 1. 根据点云更新 voxel_map _bbx_voxel_map
    // 添加以往帧的 occupied 体素
    for (size_t i = 0; i < occupied_voxels_.size(); i++)
    {
        octomap::point3d point(occupied_voxels_[i][0], occupied_voxels_[i][1], occupied_voxels_[i][2]);
        voxel_map.updateNode(point, true);
        voxel_map.search(point)->setColor(0, 0, 255);
    }
    LOG(INFO) << "past point cloud get occupancy node done" ;
    LOG(INFO) << "past octomap getNumLeafNodes: " << voxel_map.getNumLeafNodes();

    // 添加当前帧的 occupied 体素
    for (size_t i = 0; i < cloud->size(); i++)
    {

        octomap::point3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

        _bbx_voxel_map.updateNode(point, true);
        _bbx_voxel_map.search(point)->setColor(0, 0, 255);

        voxel_map.updateNode(point, true);
        voxel_map.search(point)->setColor(0, 0, 255);
    }

    LOG(INFO) << "current point cloud get occupancy node done" ;
    LOG(INFO) << "current octomap getNumLeafNodes: " << voxel_map.getNumLeafNodes();


    // 2. 如果 当前帧为第一帧，bbx进行初始化拓展
    if (is_first)
    {
        // 更新bbx_unknown_min_ 和 bbx_unknown_max_
        voxel_map.getMetricMin(bbx_unknown_min_[0], bbx_unknown_min_[1],  bbx_unknown_min_[2]);       
        voxel_map.getMetricMax(bbx_unknown_max_[0], bbx_unknown_max_[1], bbx_unknown_max_[2]);
        // 计算当前camera_pose下的z轴，为bbx扩展的方向
        Eigen::Vector3d z_axis = camera_pose.block<3,1>(0,2);
        // 计算bounding box的长度，并扩展为alpha倍
        Eigen::Vector3d bbx_length = (bbx_unknown_max_ - bbx_unknown_min_);
        Eigen::Vector3d extended_length = bbx_length * 0.5;
        // 计算扩展后的bbx
        if (z_axis[0] > 0)
        {
            bbx_unknown_max_[0] = bbx_unknown_max_[0] + extended_length[0];
        }else{
            bbx_unknown_min_[0] = bbx_unknown_min_[0] - extended_length[0];
        }
        if (z_axis[1] > 0)
        {
            bbx_unknown_max_[1] = bbx_unknown_max_[1] + extended_length[1];
        }else{
            bbx_unknown_min_[1] = bbx_unknown_min_[1] - extended_length[1];
        }
        if (z_axis[2] > 0)
        {
            bbx_unknown_max_[2] = bbx_unknown_max_[2] + extended_length[2];
        }else{
            bbx_unknown_min_[2] = bbx_unknown_min_[2] - extended_length[2];
        }
    }


    // 3. 重设置 voxel_map 
    // 3.1 重置边界
    // 边界为在上一帧中计算得出的 bbx_unknown_min_ 和 bbx_unknown_max_ 与 添加了当前帧的点云的 并集 bbx
    _bbx_voxel_map.getMetricMax(bbx_unknown_max_[0], bbx_unknown_max_[1], bbx_unknown_max_[2]);
    _bbx_voxel_map.getMetricMin(bbx_unknown_min_[0], bbx_unknown_min_[1], bbx_unknown_min_[2]);
    voxel_map.setBBXMin(to_oct3d(bbx_unknown_min_));
    voxel_map.setBBXMax(to_oct3d(bbx_unknown_max_));
    for (double x = bbx_unknown_min_[0]; x < bbx_unknown_max_[0]; x += voxel_resolution_ )
    {
        for (double y = bbx_unknown_min_[1]; y < bbx_unknown_max_[1]; y += voxel_resolution_)
        {
            for (double z = bbx_unknown_min_[2]; z < bbx_unknown_max_[2]; z += voxel_resolution_)
            {
                octomap::point3d start(x, y, z);
                octomap::point3d end(x + voxel_resolution_, y + voxel_resolution_, z + voxel_resolution_);
                voxel_map.insertRay(start, end);
                if (is_first)
                {
                    // 如果是第一帧，把_voxel_map也重置
                    _voxel_map.insertRay(start, end);
                }
                
            }
        }
    }

    // 3.2 重置 voxel 类型
    // 更新voxel_map的所有的点的颜色为黑色 表示未扫描到的点
    occupied_voxels_.resize(voxel_map.getNumLeafNodes());
    #pragma omp parallel for
    for(size_t i = 0; i < voxel_map.getNumLeafNodes(); ++i){
        octomap::ColorOcTree::leaf_iterator voxel_map_it = voxel_map.begin_leafs();
        std::advance(voxel_map_it, i);
        auto color = voxel_map_it->getColor();
        if (color.r == 0 && color.g == 0 && color.b == 255)
        {
            occupied_voxels_[i] = (Eigen::Vector3d(voxel_map_it.getX(), voxel_map_it.getY(), voxel_map_it.getZ()));
        }else{
            voxel_map_it->setColor(0, 0, 0);
            occupied_voxels_[i] = (Eigen::Vector3d(0, 0, 0));
        }
    }
    voxel_map.updateInnerOccupancy();
    // 清除occupied_voxels_中全是0的点
    occupied_voxels_.erase(std::remove(occupied_voxels_.begin(), occupied_voxels_.end(), Eigen::Vector3d(0, 0, 0)), occupied_voxels_.end());

    LOG(INFO) << "reset voxel_map bbx done" ;
    LOG(INFO) << "current octomap getNumLeafNodes: " << voxel_map.getNumLeafNodes();

    // 如果是第一帧，把_voxel_map的颜色也重置
    if (is_first)
    {
        #pragma omp parallel for
        for(size_t i = 0; i < _voxel_map.getNumLeafNodes(); ++i){
            octomap::ColorOcTree::leaf_iterator _voxel_map_it = _voxel_map.begin_leafs();
            std::advance(_voxel_map_it, i);
            _voxel_map_it->setColor(0, 0, 0);
        }
        _voxel_map.updateInnerOccupancy();
    }

    // 如果不是第一帧，遍历_voxel_map的所有的节点，把_voxel_map的节点的颜色赋值给voxel_map
    if (!is_first)
    {
        for(octomap::ColorOcTree::leaf_iterator it = _voxel_map.begin_leafs(), end=_voxel_map.end_leafs(); it!= end; ++it) {
            auto voxel_map_it = voxel_map.search(it.getX(), it.getY(), it.getZ());
            // 判断点是否在voxel_map中
            if (voxel_map_it == nullptr)
            {
                continue;
            }
            auto voxel_map_color = voxel_map_it->getColor();
            // 跳过占据体素
            if (voxel_map_color.r == 0 && voxel_map_color.g == 0 && voxel_map_color.b == 255)
            {
                continue;
            }

            auto color = it->getColor();
            voxel_map.updateNode(it.getX(), it.getY(), it.getZ(), true);
            voxel_map.search(it.getX(), it.getY(), it.getZ())->setColor(color.r, color.g, color.b);
        }
    }
    
    LOG(INFO) << "voxel map add old voxel done" ;
    LOG(INFO) << "current octomap getNumLeafNodes: " << voxel_map.getNumLeafNodes();

    // 3.3 使用光线追踪判断voxel_map被观察到的部分判断哪些voxel已经被扫描
    // 通过光线追踪判断voxel_map被观察到的部分判断哪些voxel已经被扫描
    // 以相机视锥的四个顶点为边界，以相机的光心的中心为起点，每隔一定距离就发射一条射线，计算光线与voxel_map的第一个交点
    // 换算frustum的坐标到相机的位姿
    Eigen::Vector3d p1 = camera_pose.block<3,3>(0,0) * frustum_points_[0] + camera_pose.block<3,1>(0,3);
    Eigen::Vector3d p2 = camera_pose.block<3,3>(0,0) * frustum_points_[1] + camera_pose.block<3,1>(0,3);
    Eigen::Vector3d p3 = camera_pose.block<3,3>(0,0) * frustum_points_[2] + camera_pose.block<3,1>(0,3);
    Eigen::Vector3d p4 = camera_pose.block<3,3>(0,0) * frustum_points_[3] + camera_pose.block<3,1>(0,3);
    // 计算frustum在相机坐标系下的坐标
    p1 = camera_pose.block<3,3>(0,0).transpose() * (p1 - camera_pose.block<3,1>(0,3));
    p2 = camera_pose.block<3,3>(0,0).transpose() * (p2 - camera_pose.block<3,1>(0,3));
    p3 = camera_pose.block<3,3>(0,0).transpose() * (p3 - camera_pose.block<3,1>(0,3));
    p4 = camera_pose.block<3,3>(0,0).transpose() * (p4 - camera_pose.block<3,1>(0,3));
    // 计算在水平方向的光线的数量 上取整
    int num_horizontal_rays = ceil((p2 - p1)[0] / ray_trace_step_);
    // 计算在竖直方向的光线的数量
    int num_vertical_rays = ceil((p3 - p2)[1] / ray_trace_step_);
    // 构造光线
    Eigen::Vector3d ray_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d ray_direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d ray_visual_end = Eigen::Vector3d::Zero();
    // 计算unknown_bbx的对角线的长度作为光线在bbx里面的长度
    double ray_trace_length = 1.0;
    // 定义一个二维vector来存储每一条光线与voxel_map的交点
    std::vector<std::vector<Eigen::Vector3d>> ray_hit_voxels(num_horizontal_rays*num_vertical_rays);
    // 遍历所有的光线
    int total_rays = num_horizontal_rays * num_vertical_rays;

    std::vector<octomap::point3d*> ray_end_vec(total_rays);
    std::vector<octomap::point3d*> ray_orign_vec(total_rays);
    // TODO 多线程时与单线程结果不一致
    for(size_t k = 0; k < size_t(total_rays); k++){
        int i = k / num_vertical_rays;
        int j = k % num_vertical_rays;

        ray_orign_vec[k] = nullptr;
        ray_end_vec[k] = nullptr;

        auto local_bbx_unknown_min_ = bbx_unknown_min_;
        auto local_bbx_unknown_max_ = bbx_unknown_max_;
        auto local_camera_pose = camera_pose;

        // 计算光线的方向
        ray_visual_end = Eigen::Vector3d(
            p1[0] + i*ray_trace_step_, 
            p1[1] + j*ray_trace_step_,
            p1[2]);

        // 把ray_end 换算到世界坐标系下
        ray_visual_end = local_camera_pose.block<3,3>(0,0) * ray_visual_end + local_camera_pose.block<3,1>(0,3);
        ray_origin = local_camera_pose.block<3,1>(0,3);
        ray_direction = ray_visual_end - ray_origin;
        ray_direction.normalize();

        // 计算光线与octomap边界的交点
        Eigen::Vector3d box_intersection;

        if (computeRayBoxIntersection(ray_origin, ray_direction, local_bbx_unknown_min_, local_bbx_unknown_max_, box_intersection))
        {   
            // 光线与边界相交，交点在 box_intersection 中
            // 修改光线的起点
            ray_origin = box_intersection;
            Eigen::Vector3d ray_end = ray_origin + ray_trace_length * ray_direction;
            // 计算光线与voxel_map的交点
            ray_orign_vec[k] = new octomap::point3d(ray_origin[0], ray_origin[1], ray_origin[2]);
            ray_end_vec[k] = new octomap::point3d(ray_end[0], ray_end[1], ray_end[2]);
        }
    }
    
    std::vector<octomap::KeyRay*> ray_key_vec(total_rays);
    # pragma omp parallel for
    for(int k = 0; k < total_rays; k++){
        
        // 初始化所有地址为空
        ray_key_vec[k] = nullptr;

        if (ray_orign_vec[k] == nullptr || ray_end_vec[k] == nullptr){
            continue;
        }
        // 光线与边界相交，交点在 box_intersection 中
        // 修改光线的起点
        octomap::KeyRay* ray_ptr = new octomap::KeyRay;
        // 计算光线与voxel_map的所有交点
        bool is_success = voxel_map.computeRayKeys(*ray_orign_vec[k], *ray_end_vec[k], *ray_ptr);
        if (is_success){
            ray_key_vec[k] = ray_ptr;
        }else{
            if (ray_ptr != nullptr)
            {
                delete ray_ptr;
                ray_ptr = nullptr;
            }
        }
        
        if (ray_orign_vec[k] != nullptr)
        {
           delete ray_orign_vec[k];
           ray_orign_vec[k] = nullptr;
        }
        
        if (ray_end_vec[k] != nullptr)
        {
            delete ray_end_vec[k];
            ray_end_vec[k] = nullptr;
        }
    }
    
    # pragma omp parallel for
    for (size_t k = 0; k < size_t(total_rays); ++k)
    {
        if(ray_key_vec[k] == nullptr){
            continue;
        }

        std::vector<Eigen::Vector3d> ray_trace_shot_line(ray_key_vec[k]->size());
        // 迭代遍历 ray_ptr
        auto it = ray_key_vec[k]->begin();
        for (size_t l = 0; l < ray_key_vec[k]->size(); ++l)
        {
            if (voxel_map.search(*it) != nullptr)
            {
                ray_trace_shot_line[l] = to_eigen3d(voxel_map.keyToCoord(*it));
            }
            it++;
        }
        ray_hit_voxels[k] = ray_trace_shot_line;
        if (ray_key_vec[k] != nullptr)
        {
            delete ray_key_vec[k];
            ray_key_vec[k] = nullptr;
        }
    }
        
    // 4. 计算当前视角下voxel_map的进行分类
    for (size_t i = 0; i < ray_hit_voxels.size(); i++)
    {
        // 遍历每一条光线
        bool start_occupied = false;
        for (size_t j = 0; j < ray_hit_voxels[i].size(); j++)
        {
            double x = ray_hit_voxels[i][j][0];
            double y = ray_hit_voxels[i][j][1];
            double z = ray_hit_voxels[i][j][2];
            
            auto voxel_map_it = voxel_map.search(x,y,z);
            if (voxel_map_it == nullptr)
            {
                continue;
            }
            
            auto voxel_map_color = voxel_map_it->getColor();

            if (voxel_map_it == nullptr)
            {   
                continue;
                
            }else{
                // 如果该点是占据点
                if (voxel_map_color.r == 0 && voxel_map_color.g == 0 && voxel_map_color.b == 255)
                {
                    start_occupied = true;
                
                }else{ // 如果在voxel_map中不是占据点
                    if (start_occupied)
                    {
                        auto _voxel_map_it = _voxel_map.search(x,y,z);
                        octomap::ColorOcTreeNode::Color _voxel_map_color;
                        // 且在_voxel_map中是占据点是froniter 或者是 未扫描到的点 或者是 未知点 或是不存在的点
                        // 设置为未知点
                        if (_voxel_map_it != nullptr)
                        {
                            _voxel_map_color = _voxel_map_it->getColor();
                            if ((_voxel_map_color.r == 255 && _voxel_map_color.g == 255 && _voxel_map_color.b == 255)
                                || (_voxel_map_color.r == 0 && _voxel_map_color.g == 0 && _voxel_map_color.b == 255))
                            {
                                voxel_map_it->setColor(_voxel_map_color.r, _voxel_map_color.g, _voxel_map_color.b);
                                
                            }else{
                                voxel_map_it->setColor(188, 188, 188);
                            }
                        }else{
                            voxel_map_it->setColor(188, 188, 188);
                        }
                        
                    }else{
                        // 当前点设置为empty
                        voxel_map_it->setColor(255, 255, 255);
                    }
                }
            }
        }
    }

    // 遍历voxel_map的unknown节点 对 Frontier进行类
    for(octomap::ColorOcTree::leaf_iterator it = voxel_map.begin_leafs(), end=voxel_map.end_leafs(); it!= end; ++it) {

        auto color = it->getColor();

        if (color.r != 188 || color.g != 188 || color.b != 188)
        {   
            
            continue;
        }

        // 找到该点的八个邻居
        std::vector<Eigen::Vector3d> neighbors = find_neighbors(it.getX(), it.getY(), it.getZ());
        
        // 判断该点是否是边界点
        bool is_frontier = false;
        bool has_empty_neighbor = false;
        bool has_occupied_neighbor = false;

        for (size_t i = 0; i < neighbors.size(); i++)
        {   
            double x = neighbors[i][0];
            double y = neighbors[i][1];
            double z = neighbors[i][2];
            auto nb_it = voxel_map.search(x, y, z);
            if (nb_it == nullptr)
            {
                continue;
            }
            
            auto color = nb_it->getColor();
            // 如果该点的邻居是未知点
            if (color.r == 188 && color.g == 188 && color.b == 188)
            {
                has_empty_neighbor = true;
                continue;
            }

            // 如果该点的邻居是占据点
            if (color.r == 0 && color.g == 0 && color.b == 255)
            {
                has_occupied_neighbor = true;
                continue;
            }
            
            is_frontier = has_empty_neighbor && has_occupied_neighbor;
            
            if(is_frontier){
                // 设置该点的颜色为红色
                it->setColor(255, 0, 0);
                frontier_voxels.push_back(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
                break;
            }
        }
    }

    LOG(INFO) << "voxel map classification done" ;
    LOG(INFO) << "current octomap getNumLeafNodes: " << voxel_map.getNumLeafNodes();

    // 5. 根据新的frontier_voxels和occupied_voxel 重新更新 _bbx_voxel_map

    // 5.1 更新全局的_voxel_map
    // 遍历_voxel_map的所有的节点，把voxel_map的节点的颜色赋值给_voxel_map
    for(octomap::ColorOcTree::leaf_iterator it = voxel_map.begin_leafs(), end=voxel_map.end_leafs(); it!= end; ++it) {
        auto color = it->getColor();
        _voxel_map.updateNode(it.getX(), it.getY(), it.getZ(), true);
        _voxel_map.search(it.getX(), it.getY(), it.getZ())->setColor(color.r, color.g, color.b);
    }
    _voxel_map.updateInnerOccupancy();

    _bbx_voxel_map.clear();
    // 遍历所有 Frontier 再次更新bbx的大小
    for (size_t i = 0; i < frontier_voxels.size(); ++i)
    {
        auto tmp_voxels = getSurroundingVoxels(frontier_voxels[i], surrounding_voxels_radius_);
        for (size_t j = 0; j < tmp_voxels.size(); j++)
        {   
            octomap::point3d point(tmp_voxels[j][0], tmp_voxels[j][1], tmp_voxels[j][2]);
            _bbx_voxel_map.updateNode(point, true);
            // 设置颜色为红色
            _bbx_voxel_map.search(point)->setColor(255, 0, 0);
        }
    }
    // 遍历所有的占据点
    for (size_t i = 0; i < occupied_voxels_.size(); i++)
    {
        octomap::point3d point(occupied_voxels_[i][0], occupied_voxels_[i][1], occupied_voxels_[i][2]);
        _bbx_voxel_map.updateNode(point, true);
        _bbx_voxel_map.search(point)->setColor(0, 0, 255);
    }
    _bbx_voxel_map.updateInnerOccupancy();

    //  更新bbx_unknown_min_ 和 bbx_unknown_max_
    _bbx_voxel_map.getMetricMin(bbx_unknown_min_[0], bbx_unknown_min_[1], bbx_unknown_min_[2]);
    _bbx_voxel_map.getMetricMax(bbx_unknown_max_[0], bbx_unknown_max_[1], bbx_unknown_max_[2]);

    clearOutsideBBX(voxel_map, to_oct3d(bbx_unknown_min_), to_oct3d(bbx_unknown_max_));
    voxel_map.setBBXMin(to_oct3d(bbx_unknown_min_));
    voxel_map.setBBXMax(to_oct3d(bbx_unknown_max_));
    voxel_map.updateInnerOccupancy();

    LOG(INFO) << "voxel map expand bbx done" ;
    LOG(INFO) << "current octomap getNumLeafNodes: " << voxel_map.getNumLeafNodes();
    
    output_frontier_voxels = frontier_voxels;
    output_occupied_voxels = occupied_voxels_;

    bbx_unknown_min = bbx_unknown_min_;
    bbx_unknown_max = bbx_unknown_max_;

    return voxel_map;
}


bool voxelstruct::
computeRayBoxIntersection(
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

std::vector<Eigen::Vector3d> 
voxelstruct::ray_travel(const Eigen::Vector3d ray_start, const Eigen::Vector3d ray_end){

    double _bin_size = voxel_resolution_;
    std::vector<Eigen::Vector3i> visited_voxels;
    
    // This id of the first/current voxel hit by the ray.
    // Using floor (round down) is actually very important,
    // the implicit int-casting will round up for negative numbers.
    Eigen::Vector3i current_voxel(std::floor(ray_start[0]/_bin_size),
                                    std::floor(ray_start[1]/_bin_size),
                                    std::floor(ray_start[2]/_bin_size));

    // The id of the last voxel hit by the ray.
    // TODO: what happens if the end point is on a border?
    Eigen::Vector3i last_voxel(std::floor(ray_end[0]/_bin_size),
                                std::floor(ray_end[1]/_bin_size),
                                std::floor(ray_end[2]/_bin_size));

    // Compute normalized ray direction.
    Eigen::Vector3d ray = ray_end-ray_start;
    //ray.normalize();

    // In which direction the voxel ids are incremented.
    double stepX = (ray[0] >= 0) ? 1:-1; // correct
    double stepY = (ray[1] >= 0) ? 1:-1; // correct
    double stepZ = (ray[2] >= 0) ? 1:-1; // correct

    // Distance along the ray to the next voxel border from the current position (tMaxX, tMaxY, tMaxZ).
    double next_voxel_boundary_x = (current_voxel[0]+stepX)*_bin_size; // correct
    double next_voxel_boundary_y = (current_voxel[1]+stepY)*_bin_size; // correct
    double next_voxel_boundary_z = (current_voxel[2]+stepZ)*_bin_size; // correct

    // tMaxX, tMaxY, tMaxZ -- distance until next intersection with voxel-border
    // the value of t at which the ray crosses the first vertical voxel boundary
    double tMaxX = (ray[0]!=0) ? (next_voxel_boundary_x - ray_start[0])/ray[0] : DBL_MAX; //
    double tMaxY = (ray[1]!=0) ? (next_voxel_boundary_y - ray_start[1])/ray[1] : DBL_MAX; //
    double tMaxZ = (ray[2]!=0) ? (next_voxel_boundary_z - ray_start[2])/ray[2] : DBL_MAX; //

    // tDeltaX, tDeltaY, tDeltaZ --
    // how far along the ray we must move for the horizontal component to equal the width of a voxel
    // the direction in which we traverse the grid
    // can only be FLT_MAX if we never go in that direction
    double tDeltaX = (ray[0]!=0) ? _bin_size/ray[0]*stepX : DBL_MAX;
    double tDeltaY = (ray[1]!=0) ? _bin_size/ray[1]*stepY : DBL_MAX;
    double tDeltaZ = (ray[2]!=0) ? _bin_size/ray[2]*stepZ : DBL_MAX;

    Eigen::Vector3i diff(0,0,0);
    bool neg_ray=false;
    if (current_voxel[0]!=last_voxel[0] && ray[0]<0) { diff[0]--; neg_ray=true; }
    if (current_voxel[1]!=last_voxel[1] && ray[1]<0) { diff[1]--; neg_ray=true; }
    if (current_voxel[2]!=last_voxel[2] && ray[2]<0) { diff[2]--; neg_ray=true; }
    visited_voxels.push_back(current_voxel);
    if (neg_ray) {
        current_voxel+=diff;
        visited_voxels.push_back(current_voxel);
    }

    while(last_voxel != current_voxel) {
        if (tMaxX < tMaxY) {
        if (tMaxX < tMaxZ) {
            current_voxel[0] += stepX;
            tMaxX += tDeltaX;
        } else {
            current_voxel[2] += stepZ;
            tMaxZ += tDeltaZ;
        }
        } else {
        if (tMaxY < tMaxZ) {
            current_voxel[1] += stepY;
            tMaxY += tDeltaY;
        } else {
            current_voxel[2] += stepZ;
            tMaxZ += tDeltaZ;
        }
        }
        visited_voxels.push_back(current_voxel);
    }

    std::vector<Eigen::Vector3d> visited_voxels_resolution_;

    for(size_t i = 0; i < visited_voxels.size(); i++){
        Eigen::Vector3d tmp(visited_voxels[i][0] * _bin_size,
                            visited_voxels[i][1] * _bin_size,
                            visited_voxels[i][2] * _bin_size);
        visited_voxels_resolution_.push_back(tmp);
    }

    return visited_voxels_resolution_;
}

std::vector<Eigen::Vector3d> voxelstruct::
find_neighbors(double x, double y, double z)
{
    std::vector<Eigen::Vector3d> neighbors;
    double step = voxel_resolution_;

    for (double i = x - step; i <= x + step; i += step)
    {
        for (double j = y - step; j <= y + step; j += step)
        {
            for (double k = z - step; k <= z + step; k += step)
            {
                if (i == x && j == y && k == z)
                    continue;
                neighbors.push_back(Eigen::Vector3d(i, j, k));
            }
        }
    }

    return neighbors;
}

octomap::point3d voxelstruct::
to_oct3d(const Eigen::Vector3d& v)
{
    return octomap::point3d(v.x(), v.y(), v.z());
}

Eigen::Vector3d voxelstruct::
to_eigen3d(const octomap::point3d& p)
{
    return Eigen::Vector3d(p.x(), p.y(), p.z());
}

std::vector<Eigen::Vector3d> voxelstruct::
getSurroundingVoxels(const Eigen::Vector3d& voxel, int r) {
    std::vector<Eigen::Vector3d> surrounding_voxels;

    double step = r*voxel_resolution_;

    for (double x = voxel[0] - step; x <= voxel[0] + step; x += voxel_resolution_) {
        for (double y = voxel[1] - step; y <= voxel[1] + step; y += voxel_resolution_) {
            for (double z = voxel[2] - step; z <= voxel[2] + step; z += voxel_resolution_) {
                surrounding_voxels.push_back(Eigen::Vector3d(x, y, z));
            }
        }
    }

    return surrounding_voxels;
}

void voxelstruct::clearOutsideBBX(
    octomap::ColorOcTree& map, 
    const octomap::point3d& min, 
    const octomap::point3d& max) {

    // 遍历所有的voxel_map的节点，如果节点的坐标不在min和max之间，就把该节点删除
    for(octomap::ColorOcTree::leaf_iterator it = map.begin_leafs(), end=map.end_leafs(); it!= end; ++it) {
        if (it.getX() < min.x() || it.getX() > max.x() || 
            it.getY() < min.y() || it.getY() > max.y() || 
            it.getZ() < min.z() || it.getZ() > max.z()) {
            map.deleteNode(it.getKey());
        }
    }        

    map.updateInnerOccupancy();
}