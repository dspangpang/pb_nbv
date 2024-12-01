#include "../include/pb_core/pbnbv.h"
#include "../include/pb_core/jsonparser.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Pose.h>
#include <utils_msgs/NBVTrigger.h>

#include <glog/logging.h>

class pbnbv_ros_control
{
private:
    /* data */
public:
    pbnbv_ros_control(const ros::NodeHandle& nh):pb_nbv()
    {
        this->is_first_frame_ = true;
        trigger_service = nh_.advertiseService(
            "nbv_ros_node_trigger", 
            &pbnbv_ros_control::trigger_callback, 
            this);

    }

    ~pbnbv_ros_control()=default;
    
    void run()
    {
        pb_nbv.visualization_start();
    }
private:
    ros::NodeHandle nh_;
    ros::ServiceServer trigger_service;
    pbnbv pb_nbv;

    bool is_first_frame_;

private:
    bool trigger_callback(utils_msgs::NBVTrigger::Request &req, utils_msgs::NBVTrigger::Response &res)
    {
        ROS_INFO("pbnbv_ros_node trigger service is called");

        Eigen::Matrix4d nbv;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = 
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        
        int is_terminal;

        if (is_first_frame_)
        {   
            is_first_frame_ = false;
            is_terminal = pb_nbv.execute(nbv);
            
            LOG(INFO) << "Generate first frame done ! ";
 
        }else{

            pcl::io::loadPCDFile(req.pcd_file_path, *pcl_cloud);

            // 把geometry_msgs::Pose 转换成 Eigen::Matrix4d
            Eigen::Matrix4d current_camera_pose_eigen;
            Eigen::Quaterniond current_camera_pose_quaternion(
                req.current_camera_pose.orientation.w,
                req.current_camera_pose.orientation.x,
                req.current_camera_pose.orientation.y,
                req.current_camera_pose.orientation.z);
            Eigen::Vector3d current_camera_pose_translation(
                req.current_camera_pose.position.x,
                req.current_camera_pose.position.y,
                req.current_camera_pose.position.z);
            current_camera_pose_eigen.block<3,3>(0,0) = current_camera_pose_quaternion.toRotationMatrix();
            current_camera_pose_eigen.block<3,1>(0,3) = current_camera_pose_translation;

            pb_nbv.capture(pcl_cloud, current_camera_pose_eigen);
            is_terminal = pb_nbv.execute(nbv);

            LOG(INFO) << "Generate next best view done ! ";
        }

        // 把Eigen::Matrix4d转换成geometry_msgs::Pose
        geometry_msgs::Pose camera_pose;
        Eigen::Quaterniond camera_pose_quaternion(nbv.block<3,3>(0,0));
        Eigen::Vector3d camera_pose_translation(nbv.block<3,1>(0,3));

        camera_pose.position.x = camera_pose_translation(0);
        camera_pose.position.y = camera_pose_translation(1);
        camera_pose.position.z = camera_pose_translation(2);

        camera_pose.orientation.w = camera_pose_quaternion.w();
        camera_pose.orientation.x = camera_pose_quaternion.x();
        camera_pose.orientation.y = camera_pose_quaternion.y();
        camera_pose.orientation.z = camera_pose_quaternion.z();
        
        res.nbv_camera_pose = camera_pose;
        res.is_terminated = (is_terminal == 1) ? true : false;
        res.voxel_map_size = pb_nbv.get_voxel_map_size();
        res.ellipsoids_size = pb_nbv.get_ellipsoid_size();
        res.result = 0;

        LOG(INFO) << "NBVTrigger next best view done ! ";
    
        return true;
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "pbnbv_ros_node");

    // 初始化Glog
    google::InitGoogleLogging(argv[0]);

    // 从环境变量中提取 work_dir
    std::string work_dir = std::getenv("WORK_DIR");
    if (work_dir.empty())
    {
        LOG(ERROR) << "WORK_DIR is not set !";
        return -1;
    }
    // 设置日志的输出文件
    google::SetLogDestination(google::INFO, (work_dir + "src/pb_core/log/").c_str());
    
    // 设置日志的输出级别
    FLAGS_stderrthreshold = google::INFO;

    ros::NodeHandle nh;

    // 开启多线程
    ros::AsyncSpinner spinner(1);
    spinner.start();
    pbnbv_ros_control pbnbv_ros(nh);
    pbnbv_ros.run();

}