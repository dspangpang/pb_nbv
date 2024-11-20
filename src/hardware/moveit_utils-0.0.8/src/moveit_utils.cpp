#include "../include/moveit_utils/moveit_utils.h"

// 用于定义当前 move_line 分段的状态

moveit_utils::moveit_utils(const std::string &planning_group_name) : move_group(planning_group_name) {
    
    planning_group_name_ = planning_group_name;

    //允许重新规划
    move_group.allowReplanning(true);
    // 设置最大规划时间为10秒
    move_group.setPlanningTime(10.0);

    //设置一个指向规划组的全局指针joint_model_group
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group_name_);

    // 获取robot model
    robot_model = move_group.getRobotModel();

    // 获取机械末端的名字
    end_effector_link = move_group.getEndEffectorLink();

    // 获取当前机械臂的自由度
    dof = joint_model_group->getVariableCount();

    reference_link = "default";
    
    std::cout << "end_effector_link :" << end_effector_link << std::endl;

    std::cout << "reference_link : " << reference_link << std::endl;

    std::cout << "arm dof : " << dof << std::endl;

    current_dir = ros::package::getPath("moveit_utils");

    // 获取当前机械臂的运动链
    init_manipulator_chain();

    // 判断字符串是否包含 "arm"
    if (planning_group_name_.find("arm") != std::string::npos) {
        nh_.getParam("/enable_trajectory_save", enable_trajectory_save);
        std:: cout << "enable_trajectory_save : " << enable_trajectory_save << std::endl;
        if(enable_trajectory_save){
            std::cout << "The string contains \"arm\", enable_trajectory_save is on" << std::endl;
        }else{
            std::cout << "The string contains \"arm\", enable_trajectory_save is off" << std::endl;
        }

        // 设置机械臂的速度
        double joint_speed_factor;
        nh_.getParam("/joint_speed_factor", joint_speed_factor);
        std::cout << "joint_speed_factor : " << joint_speed_factor << std::endl;
        move_group.setMaxVelocityScalingFactor(joint_speed_factor);
        move_group.setMaxAccelerationScalingFactor(joint_speed_factor);

    } else {
        enable_trajectory_save = false;
        std::cout << "The string does not contain \"arm\", enable_trajectory_save is off" << std::endl;
    }
    
}

moveit_utils::~moveit_utils() = default;

void moveit_utils::set_move_group_speed(const double &vel_factor, const double &acc_factor){

    move_group.setMaxVelocityScalingFactor(vel_factor);
    move_group.setMaxAccelerationScalingFactor(acc_factor);
    
}

std::vector<double> moveit_utils::get_current_joint_state() {

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    return joint_group_positions;
}

int moveit_utils::move_waypoints(const std::vector<geometry_msgs::Pose> &waypoints,
                                const CartesianParam &cp,
                                bool is_block) {


    return 0;
}

geometry_msgs::Pose moveit_utils::get_current_link_pose(const std::string &link_name) {

    if(reference_link == "default"){
        tf::TransformListener listener;

        std::string err;
        geometry_msgs::Pose tmp;
        tf::StampedTransform transform;

        try {
            // 等待直到可以获取到变换
            listener.waitForTransform("world", link_name, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("world", link_name, ros::Time(0), transform);

            // 将变换结果赋值给tmp
            tmp.orientation.w = transform.getRotation().w();
            tmp.orientation.x = transform.getRotation().x();
            tmp.orientation.y = transform.getRotation().y();
            tmp.orientation.z = transform.getRotation().z();

            tmp.position.x = transform.getOrigin().x();
            tmp.position.y = transform.getOrigin().y();
            tmp.position.z = transform.getOrigin().z();
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }
        
        return tmp;
    }
    else{
        tf::TransformListener listener;

        std::string err;
        geometry_msgs::Pose tmp;
        tf::StampedTransform transform;

        try {
            // 等待直到可以获取到变换
            listener.waitForTransform(reference_link, link_name, ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform(reference_link, link_name, ros::Time(0), transform);

            // 将变换结果赋值给tmp
            tmp.orientation.w = transform.getRotation().w();
            tmp.orientation.x = transform.getRotation().x();
            tmp.orientation.y = transform.getRotation().y();
            tmp.orientation.z = transform.getRotation().z();

            tmp.position.x = transform.getOrigin().x();
            tmp.position.y = transform.getOrigin().y();
            tmp.position.z = transform.getOrigin().z();
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
        }
        return tmp;
    }

}

geometry_msgs::Pose moveit_utils::get_current_end_pose() {

    return get_current_link_pose(end_effector_link);

}

int moveit_utils::write_end_pose_to_file(const std::string &path) {

    
    auto state = get_current_end_pose();
    auto position = state.position;
    auto orientation = state.orientation;

    std::vector<double> data;

    data.push_back(position.x);
    data.push_back(position.y);
    data.push_back(position.z);
    data.push_back(orientation.w);
    data.push_back(orientation.x);
    data.push_back(orientation.y);
    data.push_back(orientation.z);

    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) {
        std::cout << "Failed to open the file: " << path << std::endl;
        return -1;
    }

    // 将数据写入文件
    for (const auto& value : data) {
        file << std::fixed << std::setprecision(8) << value << " ";
    }
    file << std::endl;

    file.close();

    return 0;
}

std::string moveit_utils::getTime() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    // 转换为time_t类型
    std::time_t time_p = std::chrono::system_clock::to_time_t(now);
    // 获取毫秒部分
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    // 格式化时间
    std::tm* tm_p = std::localtime(&time_p);
    std::ostringstream oss;
    oss << std::put_time(tm_p, "%Y-%m-%d-%H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();

    return oss.str();
}

std::string moveit_utils::getTimeDir(const std::string &func) {
    
    std::string time = getTime();

    std::string path = current_dir + "/data/" + func + "_" + time;

    return path;
}


int moveit_utils::add_moveit_init_scene() {

    std::vector<moveit_msgs::CollisionObject> collision_objects;

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "elfin_base_link";
    collision_object.id = "table";

    // box默认位置
    geometry_msgs::Pose table_pose;
    table_pose.position.z = -0.01; //桌子在机械臂底座以下
    table_pose.orientation.w = 1.0;
    // box实例化 形状
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3); // 三个参数 高 1cm
    primitive.dimensions[primitive.BOX_X]=1.4;// old 1.2m
    primitive.dimensions[primitive.BOX_Y]=1.4;
    primitive.dimensions[primitive.BOX_Z]=0.01;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

   
    // 现在，添加障碍物至世界中
    planning_scene_interface.addCollisionObjects(collision_objects);

    return 0;
}

void moveit_utils::set_goal_tolerance(const double &position,
                                        const double &joints,
                                        const double &orientation){

    move_group.setGoalOrientationTolerance(orientation);
    move_group.setGoalPositionTolerance(position);
    move_group.setGoalJointTolerance(joints);
}


int moveit_utils::move_pose(const geometry_msgs::Pose &target_pose, bool is_block) {

    stateReset();

    move_group.setPoseTarget(target_pose);

    //用布尔型变量标记运动规划是否成功
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
        //打印结果
        ROS_INFO("move_pose:PLAN FAILED");
        return -1;
    }

    if(enable_trajectory_save){
        std::string dir = getTimeDir("move_pose");
        save_trajectory_to_file(dir, my_plan.trajectory_);
    }

    //打印结果
    ROS_INFO("move_pose:PLAN SUCCEED");
    moveit::core::MoveItErrorCode err;
    if (is_block) {
        err = move_group.execute(my_plan);
    } else {
        err = move_group.asyncExecute(my_plan);
    }

    int res;
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
        res = 0;
    } else {
        ROS_ERROR("move_pose:excute Falied");
        res = -1;
    }

    return res;
}

int moveit_utils::move_joint(const std::vector<double> &joint_positions_target, bool is_block) {

    stateReset();

    move_group.setJointValueTarget(joint_positions_target);

    //用布尔型变量标记运动规划是否成功
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
        //打印结果
        ROS_INFO("move_joint:PLAN FAILED");
        return -1;
    }
    //打印结果
    ROS_INFO("move_joint:PLAN SUCCEED");

   if(enable_trajectory_save){
        std::string dir = getTimeDir("move_joint");
        save_trajectory_to_file(dir, my_plan.trajectory_);
    }

    moveit::core::MoveItErrorCode err;
    if (is_block) {
        err = move_group.execute(my_plan);
    } else {
        err = move_group.asyncExecute(my_plan);
    }

    int res;
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
        res = 0;
    } else {
        ROS_ERROR("move_joint:excute Falied");
        res = -1;
    }

    return res;
}


int moveit_utils::move_line(
    const geometry_msgs::Pose &pose,
    const CartesianParam &cp,
    bool is_block){
    
    stateReset();
    
    moveit_msgs::RobotTrajectory trajectory;

    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(pose);

    bool err_try = try_compute_cartesian_path(cp, waypoints, trajectory);

    if(!err_try){

        ROS_ERROR("move_line:try_compute_cartesian_path failed");
        return -1;
    }

    bool err_trajectory = evoluate_trajectory_velocity(trajectory, cp);

    int cnt = 0;
    while(!err_trajectory)
    {
        cnt++;
        err_try = try_compute_cartesian_path(cp, waypoints, trajectory);
        err_trajectory = evoluate_trajectory_velocity(trajectory, cp);
        if(cnt > cp.trajectory_try_times){
            ROS_ERROR("move_line:evoluate_trajectory_velocityfailed");
            return -1;
        }
    }
    
    scale_trajectory_speed(trajectory, cp.speed_factor);
    my_plan.trajectory_ = trajectory;

   if(enable_trajectory_save){
        std::string dir = getTimeDir("move_line");
        save_trajectory_to_file(dir, my_plan.trajectory_);
    }

    if (is_block) {
        move_group.execute(my_plan);
    } else {
        move_group.asyncExecute(my_plan);
    }

    return 0;

}

int moveit_utils::move_line_middle_interpolation_force(
    const geometry_msgs::Pose &pose,
    const CartesianParam &cp,
    bool is_block){
    
    stateReset();
    
    auto current_state = new double(0);
    double state_step = 1.0;

    moveit_msgs::RobotTrajectory trajectory;

    auto current_pose = get_current_end_pose();

    std::vector<geometry_msgs::Pose> waypoints;

    int res = 0;

    waypoints.push_back(pose);

    auto err_try = try_compute_cartesian_path(cp, waypoints, trajectory);

    if(err_try){
         correction_trajectory_middle_interpolation_force(
                            trajectory, 
                            pose, 
                            cp,
                            current_state,
                            state_step);                                                               
    }else{
        ROS_ERROR("move_line:try_compute_cartesian_path failed");
        res = -1;
    }

    if(*current_state < 1.0){
        res = -1;
        ROS_WARN("move_line: current_state : %f", *current_state);
    }

    delete current_state;
    return res;

}

int moveit_utils::move_line_middle_interpolation(
    const geometry_msgs::Pose &pose,
    const CartesianParam &cp,
    bool is_block){
    
    stateReset();

    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::RobotTrajectory trajectory_correct;

    auto current_joint_value = get_current_joint_state();

    std::vector<geometry_msgs::Pose> waypoints;

    int err_correct;

    waypoints.push_back(pose);

    auto err_try = try_compute_cartesian_path(cp, waypoints, trajectory);

    if(err_try){
        err_correct = correction_trajectory_middle_interpolation(
                            trajectory, 
                            trajectory_correct, 
                            current_joint_value, 
                            pose, 
                            cp);                                                                

        // 修正失败就反回-1
        if(err_correct < 0){
            return -1;
        }else{
            // 评估成功 
            trajectory_correct = trajectory;
        }

    }else{
        ROS_ERROR("move_line:try_compute_cartesian_path failed");
        return -1;
    }

    scale_trajectory_speed(trajectory_correct, cp.speed_factor);
    my_plan.trajectory_ = trajectory_correct;


   if(enable_trajectory_save){
        std::string dir = getTimeDir("move_line_middle_interpolation");
        save_trajectory_to_file(dir, my_plan.trajectory_);
    }

    if (is_block) {
        move_group.execute(my_plan);
    } else {
        move_group.asyncExecute(my_plan);
    }

    return 0;

}


int moveit_utils::move_line_joint_complete(
    const geometry_msgs::Pose &pose,
    const CartesianParam &cp,
    bool is_block){

    stateReset();
    
    moveit_msgs::RobotTrajectory trajectory;

    moveit_msgs::RobotTrajectory trajectory_correct;

    auto current_pose = get_current_end_pose();

    std::vector<geometry_msgs::Pose> waypoints;

    int err_correct;

    waypoints.push_back(pose);

    auto err_try = try_compute_cartesian_path(cp, waypoints, trajectory);

    if(err_try){
        
        err_correct = correction_trajectory_joint_complete(
                            trajectory, 
                            trajectory_correct,
                            cp);                                                                

        // 修正失败就反回-1
        if(err_correct < 0){
            return -1;
        }else{
            // 评估成功 
            trajectory_correct = trajectory;
        }

    }else{
        ROS_ERROR("move_line:try_compute_cartesian_path failed");
        return -1;
    }

    scale_trajectory_speed(trajectory_correct, cp.speed_factor);
    my_plan.trajectory_ = trajectory_correct;

   if(enable_trajectory_save){
        std::string dir = getTimeDir("move_line_joint_complete");
        save_trajectory_to_file(dir, my_plan.trajectory_);
    }

    if (is_block) {
        move_group.execute(my_plan);
    } else {
        move_group.asyncExecute(my_plan);
    }

    return 0;

}

int moveit_utils::move_line_end_disturbance(
    const geometry_msgs::Pose &pose,
    const CartesianParam &cp,
    bool is_block)
{

    stateReset();

    moveit_msgs::RobotTrajectory trajectory;

    moveit_msgs::RobotTrajectory trajectory_correct;

    std::vector<geometry_msgs::Pose> waypoints;

    int err_correct;

    waypoints.push_back(pose);

    auto err_try = try_compute_cartesian_path(cp, waypoints, trajectory);

    if(err_try){
        
        err_correct = correction_trajectory_end_disturbance(
                            trajectory,
                            pose, 
                            trajectory_correct,
                            cp);                                                                

        // 修正失败就反回-1
        if(err_correct < 0){
            return -1;
        }else{
            // 评估成功 
            trajectory_correct = trajectory;
        }

    }else{
        ROS_ERROR("move_line:try_compute_cartesian_path failed");
        return -1;
    }

    scale_trajectory_speed(trajectory_correct, cp.speed_factor);
    my_plan.trajectory_ = trajectory_correct;

    if(enable_trajectory_save){
        std::string dir = getTimeDir("move_line_end_disturbance");
        save_trajectory_to_file(dir, my_plan.trajectory_);
    }

    if (is_block) {
        move_group.execute(my_plan);
    } else {
        move_group.asyncExecute(my_plan);
    }

    return 0;
}


bool moveit_utils::evoluate_trajectory_velocity(
    const moveit_msgs::RobotTrajectory &trajectory, 
    const CartesianParam &cp)
{    
    bool result = true;

    std::vector<double> end_effector_velocity(trajectory.joint_trajectory.points.size());
    std::vector<double> end_effector_velocity_diff(trajectory.joint_trajectory.points.size());


    for (int i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
    {   
        // 计算末端速度
        KDL::Jacobian jacobian;
        getEndJacobian(trajectory.joint_trajectory.points[i].positions, jacobian);

        // 把数据类型转换成eigen进行运算
        Eigen::MatrixXd eigen_jac = jacobian.data;
        std::vector<double> joint_velocity = trajectory.joint_trajectory.points[i].velocities;

        // 将std::vector映射到Eigen::VectorXd
        Eigen::Map<Eigen::MatrixXd> eigen_joint_velocity(joint_velocity.data(), joint_velocity.size(), 1);
        Eigen::MatrixXd end_effector_velocity_eigen = eigen_jac * eigen_joint_velocity;

        // 计算末端线速度的模
        double end_effector_velocity_norm = end_effector_velocity_eigen.topRows(3).norm();

        end_effector_velocity[i] = end_effector_velocity_norm;

        if (i == 0)
        {
            end_effector_velocity_diff[i] = end_effector_velocity_norm; 
        }
        else{
            end_effector_velocity_diff[i] = end_effector_velocity[i] - end_effector_velocity[i - 1];
        }
        
        // 判断末端速度是否超过阈值,不包括第一个点
        if (abs(end_effector_velocity_diff[i]) > cp.velocity_diff_torlerance && i != 0)
        {
            result = false;
            break;
        }
        
    }

    return result;
}

int moveit_utils::evoluate_trajectory_velocity(
    const moveit_msgs::RobotTrajectory &trajectory,
    std::vector<int> &error_index,
    const CartesianParam &cp)
{    

    error_index.clear();

    std::vector<double> end_effector_velocity(trajectory.joint_trajectory.points.size());
    std::vector<double> end_effector_velocity_diff(trajectory.joint_trajectory.points.size());


    for (int i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
    {   
        // 计算末端速度
        KDL::Jacobian jacobian;
        getEndJacobian(trajectory.joint_trajectory.points[i].positions, jacobian);

        // 把数据类型转换成eigen进行运算
        Eigen::MatrixXd eigen_jac = jacobian.data;
        std::vector<double> joint_velocity = trajectory.joint_trajectory.points[i].velocities;

        // 将std::vector映射到Eigen::VectorXd
        Eigen::Map<Eigen::MatrixXd> eigen_joint_velocity(joint_velocity.data(), joint_velocity.size(), 1);
        Eigen::MatrixXd end_effector_velocity_eigen = eigen_jac * eigen_joint_velocity;

        // 计算末端线速度的模
        double end_effector_velocity_norm = end_effector_velocity_eigen.topRows(3).norm();

        end_effector_velocity[i] = end_effector_velocity_norm;

        if (i == 0)
        {
            end_effector_velocity_diff[i] = end_effector_velocity_norm; 
        }
        else{
            end_effector_velocity_diff[i] = end_effector_velocity[i] - end_effector_velocity[i - 1];
        }
        
        // 判断末端速度是否超过阈值,不包括第一个点
        if (abs(end_effector_velocity_diff[i]) > cp.velocity_diff_torlerance && i != 0)
        {
            error_index.push_back(i);
        }
        
    }

    return error_index.size();
}


// 计算太过复杂,暂时不用
int
moveit_utils::evoluate_trajectory_Jacobian(
        const moveit_msgs::RobotTrajectory &trajectory,
        std::vector<KDL::Jacobian> &error_jacobian,
        std::vector<int> &error_index,
        const CartesianParam &cp){

    std::vector<double>tmp_joint_value;

    error_index.clear();
    error_jacobian.clear();

    for (int i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
    {
        tmp_joint_value.clear();
        tmp_joint_value = trajectory.joint_trajectory.points[i].positions;

        KDL::Jacobian jacobian;

        getEndJacobian(tmp_joint_value, jacobian);

        // 将 KDL::Jacobian 转换为 Eigen::MatrixXd
        Eigen::MatrixXd eigenJacobian = jacobian.data;

        // 求雅可比矩阵SVD分解的方式求解奇异值
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(eigenJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd w = svd.singularValues();

        if (w.minCoeff() < cp.singularity_threshold)
        {
            error_index.push_back(i);
            error_jacobian.push_back(jacobian);
        }
        
    }

    return error_index.size();
    
}

bool moveit_utils::is_moveing(){

    bool err = false;

    auto first_moment = move_group.getCurrentJointValues();

    usleep(1000 * 100);

    auto second_moment = move_group.getCurrentJointValues();

    for(int i = 0; i < first_moment.size(); i++){

        auto tmp = first_moment[i] - second_moment[i];

        if (tmp > 0.01){
            
            err = true;

            return err;

        }

    }

    return err;

}

void 
moveit_utils::correction_trajectory_middle_interpolation_force(
    const moveit_msgs::RobotTrajectory &trajectory,
    const geometry_msgs::Pose &target_pose,
    const CartesianParam &cp,
    double * current_state,
    const double &state_step){
    
    geometry_msgs::Pose current_pose = get_current_end_pose();

    // 评估 trajectory 如果成功直接执行，然后返回
    if(evoluate_trajectory_velocity(trajectory, cp)){
        auto local_trajectory = trajectory;
        scale_trajectory_speed(local_trajectory, cp.speed_factor);
        my_plan.trajectory_ = local_trajectory;
        if(enable_trajectory_save){
            std::string dir = getTimeDir("move_line_middle_interpolation_force");
            save_trajectory_to_file(dir, my_plan.trajectory_);
        }
        move_group.execute(my_plan);
        *current_state += state_step;
        return;
    }

    ROS_WARN("correction_trajectory_middle_interpolation:start");

    auto local_step = state_step / 2.0;
    auto mid_state = *current_state + local_step;

    // 不成功就把目标分成两段
    auto mid_pose = midpoint_interpolation(current_pose, target_pose);

    if(!is_vaild_distance(mid_pose, current_pose, cp)){
        ROS_ERROR("correction_trajectory_middle_interpolation:invaild_distance");
    }

    moveit_msgs::RobotTrajectory trajectory_front;
    moveit_msgs::RobotTrajectory trajectory_back;

    std::vector<geometry_msgs::Pose> ways_points_front;
    std::vector<geometry_msgs::Pose> ways_points_back;

    std::vector<double> tmp_joint_value;

    ways_points_front.push_back(mid_pose);
    ways_points_back.push_back(target_pose);

    // 计算前半段trajectory
    auto err_try = try_compute_cartesian_path(cp, ways_points_front, trajectory_front);
    int err_correct;

    // 如果前半段trajectory能够计算出来,就开始评估前半段的trajectory_front
    if(err_try){
        if(!evoluate_trajectory_velocity(trajectory_front, cp)){
            
            // 评估不成功 就开始修正前半段的trajectory_front 此时开始递归
            correction_trajectory_middle_interpolation_force(
                                trajectory_front,                                     
                                mid_pose, 
                                cp,
                                current_state,
                                local_step);

        }else{
            // 评估成功,直接执行      
            auto local_trajectory = trajectory_front;
            scale_trajectory_speed(local_trajectory, cp.speed_factor);
            my_plan.trajectory_ = local_trajectory;
            if(enable_trajectory_save){
                std::string dir = getTimeDir("move_line_middle_interpolation_force");
                save_trajectory_to_file(dir, my_plan.trajectory_);
            }
            move_group.execute(my_plan);
            *current_state += local_step;
       }
        
    }else{
        ROS_ERROR("correction_trajectory:try_compute_cartesian_path failed");
        return;
    }

    if (*current_state < mid_state){
        ROS_ERROR("correction_trajectory:front excute failed");
        return;
    }

    // 修改规划起点开始修正第二段trajectory
    current_pose = get_current_end_pose();
    err_try = try_compute_cartesian_path(cp, ways_points_back, trajectory_back);
    
    if(err_try){
        if(!evoluate_trajectory_velocity(trajectory_back, cp)){
            
            correction_trajectory_middle_interpolation_force(
                                trajectory_back, 
                                target_pose, 
                                cp,
                                current_state,
                                local_step);

        }else{
            // 评估成功,直接执行 
            auto local_trajectory = trajectory_back;
            scale_trajectory_speed(local_trajectory, cp.speed_factor);
            my_plan.trajectory_ = local_trajectory;
            if(enable_trajectory_save){
                std::string dir = getTimeDir("move_line_middle_interpolation_force");
                save_trajectory_to_file(dir, my_plan.trajectory_);
            }
            move_group.execute(my_plan);

            *current_state += local_step;
        }    
        
    }else{
        ROS_ERROR("correction_trajectory:try_compute_cartesian_path failed");
        return;
    }

}

int 
moveit_utils::correction_trajectory_middle_interpolation(
    const moveit_msgs::RobotTrajectory &trajectory,
    moveit_msgs::RobotTrajectory &trajectory_corrected,
    const std::vector<double> &current_joint_value,
    const geometry_msgs::Pose &target_pose,
    const CartesianParam &cp){
    
    // 评估 trajectory 如果成功直接反回
    if(evoluate_trajectory_velocity(trajectory, cp)){
        trajectory_corrected = trajectory;
        return 0;
    }
    
    geometry_msgs::Pose current_pose = compute_forward_kinematics(current_joint_value);
    std::vector<double> next_joint_value;

    // 不成功就把目标分成两段
    auto mid_pose = midpoint_interpolation(current_pose, target_pose);

    if(!is_vaild_distance(mid_pose, current_pose, cp)){
        ROS_ERROR("correction_trajectory_middle_interpolation:invaild_distance");
        return -2;
    }

    moveit_msgs::RobotTrajectory trajectory_front;
    moveit_msgs::RobotTrajectory trajectory_back;
    
    moveit_msgs::RobotTrajectory trajectory_front_corrected;
    moveit_msgs::RobotTrajectory trajectory_back_corrected;

    std::vector<geometry_msgs::Pose> ways_points_front;
    std::vector<geometry_msgs::Pose> ways_points_back;

    ways_points_front.push_back(mid_pose);
    ways_points_back.push_back(target_pose);

    // 计算前半段trajectory
    auto err_try = try_compute_cartesian_path(cp, current_joint_value, ways_points_front, trajectory_front);
    int err_correct;

    // 如果前半段trajectory能够计算出来,就开始评估前半段的trajectory_front
    if(err_try){
        if(!evoluate_trajectory_velocity(trajectory_front, cp)){
            
            // 评估不成功 就开始修正前半段的trajectory_front 此时开始递归
            err_correct = correction_trajectory_middle_interpolation(
                                trajectory_front, 
                                trajectory_front_corrected,                                        
                                current_joint_value, 
                                mid_pose, 
                                cp);

            // 修正失败就反回-1
            if(err_correct < 0){
                ROS_ERROR("correction_trajectory_middle_interpolation err");
                return -3;
            }
        }else{
            // 评估成功 
            trajectory_front_corrected = trajectory_front;
        }

        next_joint_value = trajectory_front_corrected.joint_trajectory.points.back().positions;
        
    }else{
        ROS_ERROR("correction_trajectory:try_compute_cartesian_path failed");
        return -1;
    }

    // 修改规划起点开始修正第二段trajectory
    err_try = try_compute_cartesian_path(cp, next_joint_value, ways_points_back, trajectory_back);
    
    if(err_try){
        if(!evoluate_trajectory_velocity(trajectory_back, cp)){
            
            err_correct = correction_trajectory_middle_interpolation(
                                trajectory_front, 
                                trajectory_front_corrected,                                        
                                next_joint_value, 
                                target_pose, 
                                cp);
                                
            if(err_correct < 0){
                ROS_ERROR("correction_trajectory_middle_interpolation err");
                return -3;
            }

        }else{
            // 评估成功 
            trajectory_back_corrected = trajectory_back;
        }
        
        // 两段trajectory合并
        trajectory_corrected = mergeTrajectories(trajectory_front_corrected, 
                                                trajectory_back_corrected);

        
    }else{
        ROS_ERROR("correction_trajectory:try_compute_cartesian_path failed");
        return -1;
    }

    return 0;

}


int 
moveit_utils::correction_trajectory_joint_complete(
    const moveit_msgs::RobotTrajectory &trajectory,
    moveit_msgs::RobotTrajectory &trajectory_corrected,
    const CartesianParam &cp){

    ROS_WARN("start correction_trajectory_joint_complete");

    std::vector<int> error_index;
    std::vector<Eigen::MatrixXd> error_jacobian;

    int err = evoluate_trajectory_velocity(trajectory, error_index, cp);

    trajectory_corrected = trajectory;

    int err_start_index = 0;
    int err_end_index = 0;

    std::vector<std::vector<int>> groups;

    int interval = 5;

    if(err > 0){

        ROS_WARN("the number of err points: %d", err);

        std::vector<int> tmp;
        bool is_first = true;

        // 把错误点分组
        for (int value : error_index) {
            if(is_first){
                tmp.push_back(value);
                is_first = false;
            }else{
                if(value - tmp.back() > interval){
                    groups.push_back(tmp);
                    tmp.clear();
                    tmp.push_back(value);
                }else{
                    tmp.push_back(value);
                }
            }    
        }
        groups.push_back(tmp);
        ROS_WARN("after grouped err points: %d", err);
        for(auto a : groups){
            for(auto b : a){
                ROS_WARN("%d", err);
                ROS_WARN(" ");
            }
            ROS_WARN("\r\n");
        }

        int current_ptr = 0;
        trajectory_corrected.joint_trajectory.points.clear();

        // 分出奇异点index,推断奇异位置的起点和终点,开始用joint space插值
        for (size_t i = 0; i < groups.size(); i++)
        {
            // 第一个点不会被记为错误点,所以可以直接减1不必判断超过上界
            err_start_index = groups[i].front() - 1; 
            err_end_index = (groups[i].back() == trajectory.joint_trajectory.points.size() - 1) ?
                            (trajectory.joint_trajectory.points.size() - 1) : groups[i].back();

            ROS_WARN("group %d : \r\n", err);
            ROS_WARN("err_start_index : %d \r\n", err_start_index);
            ROS_WARN("err_end_index : %d : \r\n", err_end_index);

            std::vector<double> err_start_joint_value = 
                        trajectory.joint_trajectory.points[err_start_index].positions;
            
            std::vector<double> err_end_joint_value = 
                        trajectory.joint_trajectory.points[err_end_index].positions;

            // 提取正确部分的轨迹点
            trajectory_corrected.joint_trajectory.points.insert(
                    trajectory_corrected.joint_trajectory.points.end(),
                    trajectory.joint_trajectory.points.begin() + current_ptr,
                    trajectory.joint_trajectory.points.begin() + err_start_index); 

            // 对错误区间进行按关节角度插值

            stateChange(err_start_joint_value);

            move_group.setJointValueTarget(err_end_joint_value);

            bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!success) {
                //打印结果
                ROS_ERROR("joint angular interpolation FAILED");
                return -1;
            }

            trajectory_corrected.joint_trajectory.points.insert(
                    trajectory_corrected.joint_trajectory.points.end(),
                    my_plan.trajectory_.joint_trajectory.points.begin(),
                    my_plan.trajectory_.joint_trajectory.points.end());

            current_ptr = err_end_index + 1;           

        }
        
        trajectory_corrected.joint_trajectory.points.insert(
                trajectory_corrected.joint_trajectory.points.end(),
                trajectory.joint_trajectory.points.begin() + current_ptr,
                trajectory.joint_trajectory.points.end());

        iptp_trajectory(trajectory_corrected);
    }

    return 0;

}

int 
moveit_utils::correction_trajectory_end_disturbance(
    const moveit_msgs::RobotTrajectory &trajectory,
    const geometry_msgs::Pose &target_pose,
    moveit_msgs::RobotTrajectory &trajectory_corrected,
    const CartesianParam &cp){

    std::vector<int> error_index;
    std::vector<Eigen::MatrixXd> error_jacobian;

    int err = evoluate_trajectory_velocity(trajectory, error_index, cp);

    // 出现奇异位置,对当前的末端位姿添加扰动
    double position_tolerance = 0.1;
    double angle_tolerance = 10;

    if(err > 0){
        geometry_msgs::Pose current_pose = get_current_end_pose();

        auto disturb_pose = add_pose_disturbance(
            current_pose, 
            position_tolerance,
            angle_tolerance);

        moveit_msgs::RobotTrajectory trajectory_tmp;
        std::vector<geometry_msgs::Pose> way_points_tmp;
        way_points_tmp.push_back(disturb_pose);

        auto err_try = try_compute_cartesian_path(cp, way_points_tmp, trajectory_tmp);

        if(err_try < 0){
            ROS_ERROR("correction_trajectory:try_compute_cartesian_path failed");
            return -1;
        }

        // 获取最后一个点的关节角度
        std::vector<double> tmp_joint_value = trajectory_tmp.joint_trajectory.points.back().positions;

        // 重新计算到目标的轨迹
        moveit_msgs::RobotTrajectory trajectory_tmp2;
        std::vector<geometry_msgs::Pose> way_points_tmp2;
        way_points_tmp2.push_back(target_pose);

        err_try = try_compute_cartesian_path(cp, tmp_joint_value, way_points_tmp2, trajectory_tmp2);

        if(err_try < 0){
            ROS_ERROR("correction_trajectory:try_compute_cartesian_path failed");
            return -1;
        }

        trajectory_corrected = mergeTrajectories(trajectory_tmp, trajectory_tmp2);
        
    }else{
        trajectory_corrected = trajectory;
    }

    return 0;

}


geometry_msgs::Pose 
moveit_utils::midpoint_interpolation(const geometry_msgs::Pose &current_pose,
                                        const geometry_msgs::Pose &target_pose){

    Eigen::Quaterniond current_quaternion(current_pose.orientation.w, 
                                            current_pose.orientation.x, 
                                            current_pose.orientation.y, 
                                            current_pose.orientation.z);

    Eigen::Quaterniond target_quaternion(current_pose.orientation.w, 
                                            current_pose.orientation.x, 
                                            current_pose.orientation.y, 
                                            current_pose.orientation.z);

    Eigen::Quaterniond mid_quaternion = current_quaternion.slerp(0.5, target_quaternion);

    geometry_msgs::Pose mid_pose;

    mid_pose.orientation.w = mid_quaternion.w();
    mid_pose.orientation.x = mid_quaternion.x();
    mid_pose.orientation.y = mid_quaternion.y();
    mid_pose.orientation.z = mid_quaternion.z();

    mid_pose.position.x = current_pose.position.x + 
                            (target_pose.position.x - current_pose.position.x) / 2;

    
    mid_pose.position.y = current_pose.position.y + 
                            (target_pose.position.y - current_pose.position.y) / 2; 

    
    mid_pose.position.z = current_pose.position.z + 
                            (target_pose.position.z - current_pose.position.z) / 2; 

    return mid_pose;
    
}

geometry_msgs::Pose 
moveit_utils::midpoint_interpolation_position(
    const geometry_msgs::Pose &current_pose,
    const geometry_msgs::Pose &target_pose){

    geometry_msgs::Pose mid_pose;

    mid_pose.orientation.w = target_pose.orientation.w;
    mid_pose.orientation.x = target_pose.orientation.x;
    mid_pose.orientation.y = target_pose.orientation.y;
    mid_pose.orientation.z = target_pose.orientation.z;

    mid_pose.position.x = current_pose.position.x + 
                            (target_pose.position.x - current_pose.position.x) / 2;

    
    mid_pose.position.y = current_pose.position.y + 
                            (target_pose.position.y - current_pose.position.y) / 2; 

    
    mid_pose.position.z = current_pose.position.z + 
                            (target_pose.position.z - current_pose.position.z) / 2; 

    return mid_pose;
    
}

moveit_msgs::RobotTrajectory 
moveit_utils::mergeTrajectories(const moveit_msgs::RobotTrajectory& trajectory1,
                                const moveit_msgs::RobotTrajectory& trajectory2) {
    moveit_msgs::RobotTrajectory mergedTrajectory;

    // 合并轨迹点
    mergedTrajectory.joint_trajectory = trajectory1.joint_trajectory;
    mergedTrajectory.joint_trajectory.points.insert(mergedTrajectory.joint_trajectory.points.end(),
                                                    trajectory2.joint_trajectory.points.begin(),
                                                    trajectory2.joint_trajectory.points.end());

    return mergedTrajectory;
}

bool 
moveit_utils::try_compute_cartesian_path(const CartesianParam &cp,
                                        const std::vector<geometry_msgs::Pose> &waypoints,
                                        moveit_msgs::RobotTrajectory &trajectory) {
    
    bool success = false;

    // true代表避障规划
    for(int i = 0; i < cp.try_times; i++){

        double fraction = move_group.computeCartesianPath(waypoints,
                                                            cp.eef_step,
                                                            cp.jump_threshold,
                                                            trajectory,
                                                            true);

        success = (fraction >= 1.0);

        if(success){
            break;
        }
    }
    
    return success;

}


bool 
moveit_utils::try_compute_cartesian_path(const CartesianParam &cp,
                                        const std::vector<double> start_joint_value,
                                        const std::vector<geometry_msgs::Pose> &waypoints,
                                        moveit_msgs::RobotTrajectory &trajectory) {
    
    stateChange(start_joint_value);

    bool success = false;

    // true代表避障规划
    for(int i = 0; i < cp.try_times; i++){

        double fraction = move_group.computeCartesianPath(waypoints,
                                                            cp.eef_step,
                                                            cp.jump_threshold,
                                                            trajectory,
                                                            true);

        success = (fraction >= 1.0);

        if(success){
            break;
        }
    }
        
    return success;

}

bool moveit_utils::is_vaild_distance(const geometry_msgs::Pose& a,
                                    const geometry_msgs::Pose& b,
                                    const CartesianParam &cp) {

    
    Eigen::Vector3d v1(a.position.x, a.position.y, a.position.z);
    Eigen::Vector3d v2(b.position.x, b.position.y, b.position.z);
    double dis = (v1-v2).norm();

    return dis >= cp.eef_step;

}


void moveit_utils::set_reference_link(const std::string &reference_link){

    this->reference_link = reference_link;

    std::cout << "reference_link update to :" << this->reference_link << std::endl;

}

bool moveit_utils::getEndJacobian(
    const std::vector<double> &joint_values,
    KDL::Jacobian &jacobian){


    KDL::ChainJntToJacSolver jac_solver(control_chain);  // 雅可比矩阵求解器
    KDL::Jacobian jacobian_local(control_chain.getNrOfJoints());
    KDL::JntArray joint_positions(dof);

    // 将关节角度转换为KDL库中的关节数组
    for (int i = 0; i < dof; i++)
    {
        joint_positions(i) = joint_values[i];
    }

    auto result = jac_solver.JntToJac(joint_positions, jacobian_local, -1);  // 计算雅可比矩阵

    if (result < 0) {

        jac_solver.getError();
        std::cerr << "Failed to compute Jacobian!" << std::endl;
        return false;
    } else {
        // 雅可比矩阵现在包含了机械臂的关节速度与末端速度的关系
    }

    jacobian = jacobian_local;

    return true;

}



void moveit_utils::stateReset(){

    move_group.setStartStateToCurrentState();
    
}

void moveit_utils::stateChange(const std::vector<double> &joint_values){
    

    moveit::core::RobotStatePtr start_state(move_group.getCurrentState());
    
    start_state->setJointGroupPositions(joint_model_group, joint_values);
    
    move_group.setStartState(*start_state);

}


geometry_msgs::Pose
moveit_utils::add_pose_disturbance(
    const geometry_msgs::Pose &input_pose,
    const double position_tolerance,
    const double angle_tolerance){

    geometry_msgs::Pose output_pose;

    double rad_tolerance = angle_tolerance * M_PI / 180;

    // 生成随机数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-position_tolerance, position_tolerance);
    std::uniform_real_distribution<> ang(-rad_tolerance, rad_tolerance);

    // 生成三个dis随机数
    double random_dis1 = dis(gen);
    double random_dis2 = dis(gen);
    double random_dis3 = dis(gen);

    // 生成三个ang随机数
    double random_ang1 = ang(gen);
    double random_ang2 = ang(gen);
    double random_ang3 = ang(gen);
    
    // 根据 position_tolerance 添加随机位置扰动
    output_pose.position.x = input_pose.position.x + random_dis1;
    output_pose.position.y = input_pose.position.y + random_dis2;
    output_pose.position.z = input_pose.position.z + random_dis3;
   

    // 根据 angle_tolerance 添加随机姿态扰动
    Eigen::Quaterniond current_quaternion(input_pose.orientation.w, 
                                            input_pose.orientation.x, 
                                            input_pose.orientation.y, 
                                            input_pose.orientation.z);

    auto current_euler = current_quaternion.toRotationMatrix().eulerAngles(2, 1, 0);

    current_euler[0] += random_ang1;
    current_euler[1] += random_ang2;
    current_euler[2] += random_ang3;

    Eigen::Quaterniond output_quaternion = 
        Eigen::AngleAxisd(current_euler(2), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(current_euler(1), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(current_euler(0), Eigen::Vector3d::UnitX());

    output_pose.orientation.w = output_quaternion.w();
    output_pose.orientation.x = output_quaternion.x();
    output_pose.orientation.y = output_quaternion.y();
    output_pose.orientation.z = output_quaternion.z();

    return output_pose;

}


void moveit_utils::iptp_trajectory(moveit_msgs::RobotTrajectory& trajectory){
    
    robot_trajectory::RobotTrajectory rt(
    move_group.getCurrentState()->getRobotModel(), 
    planning_group_name_);
    
    rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    iptp.computeTimeStamps(rt, 1, 1);

    rt.getRobotTrajectoryMsg(trajectory);
}

void 
moveit_utils::scale_trajectory_speed(moveit_msgs::RobotTrajectory& trajectory,const double &scale){
    
    int n_joints = trajectory.joint_trajectory.joint_names.size();
    for(int i = 0; i < trajectory.joint_trajectory.points.size(); ++i){
        trajectory.joint_trajectory.points[i].time_from_start *= 1/scale;
        for (int j = 0; j < n_joints; j++)
        {
            trajectory.joint_trajectory.points[i].velocities[j] *=scale;
            trajectory.joint_trajectory.points[i].accelerations[j] *= scale* scale;
        }
        
    }
}

geometry_msgs::Pose 
moveit_utils::compute_forward_kinematics(const std::vector<double> joint_values)
{
    // 创建正向运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver(control_chain);

    KDL::JntArray joint_positions(dof);
    
    // 将关节角度转换为KDL库中的关节数组
    for (int i = 0; i < dof; i++)
    {
        joint_positions(i) = joint_values[i];
    }

    // 正向运动学计算
    KDL::Frame end_pose;
    geometry_msgs::Pose out_end_pose;
    fk_solver.JntToCart(joint_positions, end_pose);

    // 输出末端执行器的位姿
    out_end_pose.position.x = end_pose.p.x();
    out_end_pose.position.y = end_pose.p.y();
    out_end_pose.position.z = end_pose.p.z();

    end_pose.M.GetQuaternion(
        out_end_pose.orientation.x,
        out_end_pose.orientation.y,
        out_end_pose.orientation.z,
        out_end_pose.orientation.w);

    return out_end_pose;
}

void moveit_utils::init_manipulator_chain()
{
    // 创建正向运动学求解器

    // 获取完整的运动链
    const std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    const double error = 1e-5;
    double timeout = 0.05;
    std::string chain_start, chain_end, urdf_param;

    chain_start = link_names.front();
    chain_end = link_names.back();
    urdf_param = "/robot_description";
    TRAC_IK::TRAC_IK ik_solver(chain_start, chain_end, urdf_param, timeout, error, TRAC_IK::Speed);

    bool valid = ik_solver.getKDLChain(control_chain);
    
    if (!valid) {
        ROS_ERROR("There was no valid KDL chain found");
    }
}

int moveit_utils::save_trajectory_to_file(const std::string &path, const moveit_msgs::RobotTrajectory &trajectory)
{

    std::string pose_path = path + "/" + "pose_point.txt";

    std::string pose_velocity_path = path + "/" + "pose_velocity.txt";

    std::string cmd = "mkdir " + path;
    system(cmd.c_str());

    cmd = "touch " + pose_path;
    system(cmd.c_str());

    cmd = "touch " + pose_velocity_path;
    system(cmd.c_str());

    std::ofstream outfile_pose_path;

    std::ofstream outfile_velocity_path;
    
    outfile_pose_path.open(pose_path.c_str(), std::ios::out);

    outfile_velocity_path.open(pose_velocity_path.c_str(), std::ios::out);

    if (!outfile_pose_path.is_open())
    {
        std::cerr << "Failed to open file: " << pose_path << std::endl;
        return -1;
    }

    if (!outfile_velocity_path.is_open())
    {
        std::cerr << "Failed to open file: " << pose_velocity_path << std::endl;
        return -1;
    }

    // 保存末端位姿数据

    for (int i = 0; i < trajectory.joint_trajectory.points.size(); ++i)
    {   
        // 计算末端位姿
        std::vector<double> joint_valuse = trajectory.joint_trajectory.points[i].positions;
        geometry_msgs::Pose point_pose = compute_forward_kinematics(joint_valuse);

        // 保存末端位姿数据
        outfile_pose_path << point_pose.position.x << " " 
                            << point_pose.position.y << " " 
                            << point_pose.position.z << " " 
                            << point_pose.orientation.w << " "
                            << point_pose.orientation.x << " "
                            << point_pose.orientation.y << " "
                            << point_pose.orientation.z << std::endl;
        
        // 计算末端速度
        KDL::Jacobian jacobian;
        getEndJacobian(trajectory.joint_trajectory.points[i].positions, jacobian);

        // 把数据类型转换成eigen进行运算
        Eigen::MatrixXd eigen_jac = jacobian.data;
        std::vector<double> joint_velocity = trajectory.joint_trajectory.points[i].velocities;

        // 将std::vector映射到Eigen::VectorXd
        Eigen::Map<Eigen::MatrixXd> eigen_joint_velocity(joint_velocity.data(), joint_velocity.size(), 1);

        Eigen::MatrixXd end_effector_velocity = eigen_jac * eigen_joint_velocity;

        // 保存末端速度

        // 保存末端位姿数据
        for (size_t i = 0; i < end_effector_velocity.size(); ++i)
        {
            outfile_velocity_path << end_effector_velocity(i, 0);
            if (i == end_effector_velocity.size() - 1)
            {
                outfile_velocity_path << std::endl;
            }else{
                outfile_velocity_path << " ";
            }
            
        }
        
    }

    // 关闭文件
    outfile_pose_path.close();
    outfile_velocity_path.close();

    return 0;
}
