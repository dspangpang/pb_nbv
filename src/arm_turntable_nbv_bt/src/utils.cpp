#include "../include/arm_turntable_nbv_bt/utils.h"

geometry_msgs::Pose Turntable_NBV::analyseTargetPose(const std::string &target_pose){
        
    geometry_msgs::Pose current_arm_target_pose;
    size_t start;
    size_t end;

    start = target_pose.find_first_of("[");
    end = target_pose.find_first_of("]");

    std::string tmp = target_pose.substr(start + 1 , end - start - 1);

    std::istringstream istr1(tmp);

    std::vector<double> data;

    double tmp_data;
    while (istr1 >> std::setprecision(8) >> tmp_data)
    {
        data.push_back(tmp_data);
    }

    if (data.size() != 7)
    {   
        ROS_ERROR("Get wrong arm target pose parameter %s !!!", target_pose.c_str());
        ROS_ERROR("Current size is %ld, but it should be 7 !!!", data.size());
        return current_arm_target_pose;
    }

    current_arm_target_pose.position.x = data[0];
    current_arm_target_pose.position.y = data[1];
    current_arm_target_pose.position.z = data[2];
    current_arm_target_pose.orientation.w = data[3];
    current_arm_target_pose.orientation.x = data[4];
    current_arm_target_pose.orientation.y = data[5];
    current_arm_target_pose.orientation.z = data[6];

    return current_arm_target_pose;
}

std::vector<double> Turntable_NBV::analyseJointValue(const std::string &joint_value){
        
    std::vector<double> current_arm_joint_value;
    size_t start;
    size_t end;

    start = joint_value.find_first_of("[");
    end = joint_value.find_first_of("]");

    std::string tmp = joint_value.substr(start + 1 , end - start - 1);

    std::istringstream istr1(tmp);  

    double tmp_data;
    while (istr1 >> std::setprecision(8) >> tmp_data)
    {
        current_arm_joint_value.push_back(tmp_data);
    }

    return current_arm_joint_value;
}

std::string Turntable_NBV::jointValueTostring(const std::vector<double> &joint_value){
    std::string joint_value_str;
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < joint_value.size(); i++)
    {   
        if (i < joint_value.size() - 1)
        {
            ss << joint_value[i] << " ";
        }else{
            ss << joint_value[i];
        }
        
        
    }
    ss << "]";
    joint_value_str = ss.str();
    return joint_value_str;
}

std::vector<double> Turntable_NBV::analyseDataArry(const std::string &joint_value){
        
    std::vector<double> data_arry;
    size_t start;
    size_t end;

    start = joint_value.find_first_of("[");
    end = joint_value.find_first_of("]");

    std::string tmp = joint_value.substr(start + 1 , end - start - 1);

    std::istringstream istr1(tmp);  

    double tmp_data;
    while (istr1 >> std::setprecision(8) >> tmp_data)
    {
        data_arry.push_back(tmp_data);
    }

    return data_arry;
}

Eigen::Matrix4d Turntable_NBV::poseToMatrix(const geometry_msgs::Pose& pose)
{
    Eigen::Matrix4d matrix;
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    matrix.setIdentity();
    matrix.block<3,3>(0,0) = q.toRotationMatrix();
    matrix(0,3) = pose.position.x;
    matrix(1,3) = pose.position.y;
    matrix(2,3) = pose.position.z;
    return matrix;
}

geometry_msgs::Pose Turntable_NBV::matrixToPose(const Eigen::Matrix4d& matrix)
{
    geometry_msgs::Pose pose;
    Eigen::Quaterniond q(matrix.block<3,3>(0,0));
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.position.x = matrix(0,3);
    pose.position.y = matrix(1,3);
    pose.position.z = matrix(2,3);
    return pose;
}

std::string Turntable_NBV::poseToString(const geometry_msgs::Pose& pose)
{
    std::string pose_str;
    std::stringstream ss;
    ss << "[" << pose.position.x << " " << pose.position.y << " " << pose.position.z << 
    " " << pose.orientation.w << " " << pose.orientation.x << " " << pose.orientation.y << 
    " " << pose.orientation.z << "]";
    pose_str = ss.str();
    return pose_str;
}

double Turntable_NBV::get_angel(const geometry_msgs::Pose current_pos,
                        const geometry_msgs::Pose target_pos)
{

    // 第一个向量
    Eigen::Vector2d v1(current_pos.position.x, current_pos.position.y);

    // 第二个向量  
    Eigen::Vector2d v2(target_pos.position.x, target_pos.position.y);

    // 计算夹角  
    double angle = std::atan2(v2.y(), v2.x()) - std::atan2(v1.y(), v1.x());

    // 保证angel在0~2*M_PI之间
    if(angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    else if(angle > M_PI)
    {
        angle -= 2 * M_PI;
    }

    return angle;

}

void Turntable_NBV::print_error(const std::string& err_msg){
    std::cout << "\033[31m" << err_msg << "\033[0m" << std::endl;
}
void Turntable_NBV::print_warn(const std::string& warn_msg){
    std::cout << "\033[33m" << warn_msg << "\033[0m" << std::endl;
}
void Turntable_NBV::print_success(const std::string& success_msg){
    std::cout << "\033[32m" << success_msg << "\033[0m" << std::endl;
}

bool Turntable_NBV::callGetPose(ros::NodeHandle &nh,const std::string &link_name,geometry_msgs::Pose &pose)
{
    // 获取机械臂末端位姿
    ros::ServiceClient client = nh.serviceClient<utils_msgs::GetPose>("GetPose");
    utils_msgs::GetPose srv;
    srv.request.link_name = link_name;// "dummy_gripper"
    if(client.call(srv)){
        pose = srv.response.pose;
        return true;
    }
    else{
        return false;
    }
}

bool Turntable_NBV::callGetJointValue(ros::NodeHandle &nh, std::vector<double> &joint_value)
{
    // 获取机械臂关节角
    ros::ServiceClient client = nh.serviceClient<utils_msgs::GetJointValue>("GetJointValue");
    utils_msgs::GetJointValue srv;
    if(client.call(srv)){
        joint_value = srv.response.joint_value;
        return true;
    }
    else{
        return false;
    }
}
