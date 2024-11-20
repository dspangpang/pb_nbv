#pragma once
#include <unordered_set>
#include <algorithm>
#include <iostream>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif
#include <fstream>  
#include <string>  
#include <vector> 
#include <thread>
#include <chrono>
#include <atomic>
#include <ctime> 
#include <cmath>
#include <mutex>
#include <map>
#include <set>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
// #include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

using namespace std;

/** \brief Distortion model: defines how pixel coordinates should be mapped to sensor coordinates. */
typedef enum rs2_distortion
{
	RS2_DISTORTION_NONE, /**< Rectilinear images. No distortion compensation required. */
	RS2_DISTORTION_MODIFIED_BROWN_CONRADY, /**< Equivalent to Brown-Conrady distortion, except that tangential distortion is applied to radially distorted points */
	RS2_DISTORTION_INVERSE_BROWN_CONRADY, /**< Equivalent to Brown-Conrady distortion, except undistorts image instead of distorting it */
	RS2_DISTORTION_FTHETA, /**< F-Theta fish-eye distortion model */
	RS2_DISTORTION_BROWN_CONRADY, /**< Unmodified Brown-Conrady distortion model */
	RS2_DISTORTION_KANNALA_BRANDT4, /**< Four parameter Kannala Brandt distortion model */
	RS2_DISTORTION_COUNT                   /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_distortion;

/** \brief Video stream intrinsics. */
typedef struct rs2_intrinsics
{
	int           width;     /**< Width of the image in pixels */
	int           height;    /**< Height of the image in pixels */
	float         ppx;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
	float         ppy;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
	float         fx;        /**< Focal length of the image plane, as a multiple of pixel width */
	float         fy;        /**< Focal length of the image plane, as a multiple of pixel height */
	rs2_distortion model;    /**< Distortion model of the image */
	float         coeffs[5]; /**< Distortion coefficients */
} rs2_intrinsics;

/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera */
static void rs2_project_point_to_pixel(float pixel[2], const struct rs2_intrinsics* intrin, const float point[3])
{
	float x = point[0] / point[2], y = point[1] / point[2];

	if ((intrin->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY) ||
		(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY))
	{

		float r2 = x * x + y * y;
		float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;
		x *= f;
		y *= f;
		float dx = x + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
		float dy = y + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);
		x = dx;
		y = dy;
	}
	if (intrin->model == RS2_DISTORTION_FTHETA)
	{
		float r = sqrtf(x * x + y * y);
		if (r < FLT_EPSILON)
		{
			r = FLT_EPSILON;
		}
		float rd = (float)(1.0f / intrin->coeffs[0] * atan(2 * r * tan(intrin->coeffs[0] / 2.0f)));
		x *= rd / r;
		y *= rd / r;
	}
	if (intrin->model == RS2_DISTORTION_KANNALA_BRANDT4)
	{
		float r = sqrtf(x * x + y * y);
		if (r < FLT_EPSILON)
		{
			r = FLT_EPSILON;
		}
		float theta = atan(r);
		float theta2 = theta * theta;
		float series = 1 + theta2 * (intrin->coeffs[0] + theta2 * (intrin->coeffs[1] + theta2 * (intrin->coeffs[2] + theta2 * intrin->coeffs[3])));
		float rd = theta * series;
		x *= rd / r;
		y *= rd / r;
	}

	pixel[0] = x * intrin->fx + intrin->ppx;
	pixel[1] = y * intrin->fy + intrin->ppy;
}

/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
static void rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics* intrin, const float pixel[2], float depth)
{
	assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
	//assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

	float x = (pixel[0] - intrin->ppx) / intrin->fx;
	float y = (pixel[1] - intrin->ppy) / intrin->fy;
	if (intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
	{
		float r2 = x * x + y * y;
		float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2 * r2 + intrin->coeffs[4] * r2 * r2 * r2;
		float ux = x * f + 2 * intrin->coeffs[2] * x * y + intrin->coeffs[3] * (r2 + 2 * x * x);
		float uy = y * f + 2 * intrin->coeffs[3] * x * y + intrin->coeffs[2] * (r2 + 2 * y * y);
		x = ux;
		y = uy;
	}
	if (intrin->model == RS2_DISTORTION_KANNALA_BRANDT4)
	{
		float rd = sqrtf(x * x + y * y);
		if (rd < FLT_EPSILON)
		{
			rd = FLT_EPSILON;
		}

		float theta = rd;
		float theta2 = rd * rd;
		for (int i = 0; i < 4; i++)
		{
			float f = theta * (1 + theta2 * (intrin->coeffs[0] + theta2 * (intrin->coeffs[1] + theta2 * (intrin->coeffs[2] + theta2 * intrin->coeffs[3])))) - rd;
			if (abs(f) < FLT_EPSILON)
			{
				break;
			}
			float df = 1 + theta2 * (3 * intrin->coeffs[0] + theta2 * (5 * intrin->coeffs[1] + theta2 * (7 * intrin->coeffs[2] + 9 * theta2 * intrin->coeffs[3])));
			theta -= f / df;
			theta2 = theta * theta;
		}
		float r = tan(theta);
		x *= r / rd;
		y *= r / rd;
	}
	if (intrin->model == RS2_DISTORTION_FTHETA)
	{
		float rd = sqrtf(x * x + y * y);
		if (rd < FLT_EPSILON)
		{
			rd = FLT_EPSILON;
		}
		float r = (float)(tan(intrin->coeffs[0] * rd) / atan(2 * tan(intrin->coeffs[0] / 2.0f)));
		x *= r / rd;
		y *= r / rd;
	}

	point[0] = depth * x;
	point[1] = depth * y;
	point[2] = depth;
}

#define MCMF 0
#define OA 1
#define UV 2
#define RSE 3
#define APORA 4
#define Kr 5
#define NBVNET 6
#define SCVP 7

class Share_Data
{
public:
	//可变输入参数
	string pcd_file_path;
	string yaml_file_path;
	string name_of_pcd;
	string nbv_net_path;
	string sc_net_path;

	int num_of_views;					//一次采样视点个数
	double cost_weight;
	rs2_intrinsics color_intrinsics;
	double depth_scale;

	//运行参数
	int process_cnt;					//过程编号
	atomic<double> pre_clock;			//系统时钟
	atomic<bool> over;					//过程是否结束
	bool show;
	int num_of_max_iteration;

	//点云数据
	atomic<int> vaild_clouds;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;							//点云组
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcd;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground_truth;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final;
	bool move_wait;

	//八叉地图
	octomap::ColorOcTree* octo_model;
	octomap::ColorOcTree* cloud_model;
	octomap::ColorOcTree* ground_truth_model;
	octomap::ColorOcTree* GT_sample;
	double octomap_resolution;
	double ground_truth_resolution;
	double map_size;
	double p_unknown_upper_bound; //! Upper bound for voxels to still be considered uncertain. Default: 0.97.
	double p_unknown_lower_bound; //! Lower bound for voxels to still be considered uncertain. Default: 0.12.
	
	//工作空间与视点空间
	atomic<bool> now_view_space_processed;
	atomic<bool> now_views_infromation_processed;
	atomic<bool> move_on;

	Eigen::Matrix4d now_camera_pose_world;
	Eigen::Vector3d object_center_world;		//物体中心
	double predicted_size;						//物体BBX半径

	int method_of_IG;
	pcl::visualization::PCLVisualizer::Ptr viewer;

	double stop_thresh_map;
	double stop_thresh_view;

	double skip_coefficient;

	double sum_local_information;
	double sum_global_information;

	double sum_robot_cost;
	double camera_to_object_dis;
	double move_weight;
	bool robot_cost_negtive;
	bool move_cost_on;

	int num_of_max_flow_node;
	double interesting_threshold;

	double see_threshold;
	double need_threshold;

	int init_voxels;     //点云voxel个数
	int voxels_in_BBX;   //地图voxel个数
	double init_entropy; //地图信息熵
	double movement_cost = 0; //总运动代价

	string save_path;

	vector<int> view_label_id;
	double min_z_table;

	Share_Data(string _config_file_path,string test_name="")
	{
		process_cnt = -1;
		yaml_file_path = _config_file_path;
		//读取yaml文件
		cv::FileStorage fs;
		fs.open(yaml_file_path, cv::FileStorage::READ);
		fs["model_path"] >> pcd_file_path;
		fs["name_of_pcd"] >> name_of_pcd;
		fs["method_of_IG"] >> method_of_IG;
		fs["octomap_resolution"] >> octomap_resolution;
		fs["ground_truth_resolution"] >> ground_truth_resolution;
		fs["num_of_max_iteration"] >> num_of_max_iteration;
		fs["show"] >> show;
		fs["move_wait"] >> move_wait;
		fs["nbv_net_path"] >> nbv_net_path;
		fs["sc_net_path"] >> sc_net_path;
		fs["p_unknown_upper_bound"] >> p_unknown_upper_bound;
		fs["p_unknown_lower_bound"] >> p_unknown_lower_bound;
		fs["num_of_views"] >> num_of_views;
		fs["move_cost_on"] >> move_cost_on;
		fs["move_weight"] >> move_weight;
		fs["cost_weight"] >> cost_weight;
		fs["robot_cost_negtive"] >> robot_cost_negtive;
		fs["skip_coefficient"] >> skip_coefficient;
		fs["num_of_max_flow_node"] >> num_of_max_flow_node;
		fs["interesting_threshold"] >> interesting_threshold;
		fs["see_threshold"] >> see_threshold;
		fs["need_threshold"] >> need_threshold;
		fs["color_width"] >> color_intrinsics.width;
		fs["color_height"] >> color_intrinsics.height;
		fs["color_fx"] >> color_intrinsics.fx;
		fs["color_fy"] >> color_intrinsics.fy;
		fs["color_ppx"] >> color_intrinsics.ppx;
		fs["color_ppy"] >> color_intrinsics.ppy;
		fs["color_model"] >> color_intrinsics.model;
		fs["color_k1"] >> color_intrinsics.coeffs[0];
		fs["color_k2"] >> color_intrinsics.coeffs[1];
		fs["color_k3"] >> color_intrinsics.coeffs[2];
		fs["color_p1"] >> color_intrinsics.coeffs[3];
		fs["color_p2"] >> color_intrinsics.coeffs[4];
		fs["depth_scale"] >> depth_scale;
		fs.release();
		if (test_name != "") name_of_pcd = test_name;
		//读取转换后模型的pcd文件
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pcd(new pcl::PointCloud<pcl::PointXYZ>);
		cloud_pcd = temp_pcd;
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path + name_of_pcd + ".pcd", *cloud_pcd) == -1) {
			cout << "Can not read 3d model file. Check." << endl;
		}
		octo_model = new octomap::ColorOcTree(octomap_resolution);
		//octo_model->setProbHit(0.95);	//设置传感器命中率,初始0.7
		//octo_model->setProbMiss(0.05);	//设置传感器失误率，初始0.4
		//octo_model->setClampingThresMax(1.0);	//设置地图节点最大值，初始0.971
		//octo_model->setClampingThresMin(0.0);	//设置地图节点最小值，初始0.1192
		octo_model->setOccupancyThres(0.65);	//设置节点占用阈值，初始0.5
		ground_truth_model = new octomap::ColorOcTree(ground_truth_resolution);
		//ground_truth_model->setProbHit(0.95);	//设置传感器命中率,初始0.7
		//ground_truth_model->setProbMiss(0.05);	//设置传感器失误率，初始0.4
		//ground_truth_model->setClampingThresMax(1.0);	//设置地图节点最大值，初始0.971
		//ground_truth_model->setClampingThresMin(0.0);	//设置地图节点最小值，初始0.1192
		GT_sample = new octomap::ColorOcTree(octomap_resolution);
		//GT_sample->setProbHit(0.95);	//设置传感器命中率,初始0.7
		//GT_sample->setProbMiss(0.05);	//设置传感器失误率，初始0.4
		//GT_sample->setClampingThresMax(1.0);	//设置地图节点最大值，初始0.971
		//GT_sample->setClampingThresMin(0.0);	//设置地图节点最小值，初始0.1192
		cloud_model = new octomap::ColorOcTree(ground_truth_resolution);
		//cloud_model->setProbHit(0.95);	//设置传感器命中率,初始0.7
		//cloud_model->setProbMiss(0.05);	//设置传感器失误率，初始0.4
		//cloud_model->setClampingThresMax(1.0);	//设置地图节点最大值，初始0.971
		//cloud_model->setClampingThresMin(0.0);	//设置地图节点最小值，初始0.1192
		cloud_model->setOccupancyThres(0.65);	//设置节点占用阈值，初始0.5*/
		if (num_of_max_flow_node == -1) num_of_max_flow_node = num_of_views;
		now_camera_pose_world = Eigen::Matrix4d::Identity(4, 4);
		over = false;
		pre_clock = clock();
		vaild_clouds = 0;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_final = temp;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_gt(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud_ground_truth = temp_gt;
		save_path = "/root/work_place/pb_nbv/src/scvp_core/cache/" + name_of_pcd + '_' + to_string(method_of_IG);
		if (move_cost_on == true) save_path += '_' + to_string(move_weight);
		cout << "pcd and yaml files readed." << endl;
		cout << "save_path is: " << save_path << endl;
		srand(clock());
	}

	~Share_Data() {
		delete octo_model;
		delete ground_truth_model;
		delete GT_sample;
		cloud_pcd->~PointCloud();
		cloud_final->~PointCloud();
		cloud_ground_truth->~PointCloud();
		for (int i = 0; i < clouds.size(); i++)
			clouds[i]->~PointCloud();
		if (show) viewer->~PCLVisualizer();
	}

	double out_clock()
	{   //返回用时，并更新时钟
		double now_clock = clock();
		double time_len = now_clock - pre_clock;
		pre_clock = now_clock;
		return time_len;
	}

	void access_directory(string cd)
	{   //检测多级目录的文件夹是否存在，不存在就创建
#ifdef _WIN32
		string temp;
		for (int i = 0; i < cd.length(); i++)
			if (cd[i] == '/') {
				if (access(temp.c_str(), 0) != 0) mkdir(temp.c_str());
				temp += cd[i];
			}
			else temp += cd[i];
		if (access(temp.c_str(), 0) != 0) mkdir(temp.c_str());
#else
		string temp;
		for (int i = 0; i < cd.length(); i++)
			if (cd[i] == '/') {
				if (access(temp.c_str(), 0) != 0) mkdir(temp.c_str(), 0777);
				temp += cd[i];
			}
			else temp += cd[i];
		if (access(temp.c_str(), 0) != 0) mkdir(temp.c_str(), 0777);
		std::cout << "mkdir " << temp << std::endl;
#endif
	}

	void save_posetrans_to_disk(Eigen::Matrix4d& T, string cd, string name, int frames_cnt)
	{   //存放旋转矩阵数据至磁盘
		std::stringstream pose_stream, path_stream;
		std::string pose_file, path;
		path_stream << "../data" << "_" << process_cnt << cd;
		path_stream >> path;
		access_directory(path);
		pose_stream << "../data" << "_" << process_cnt << cd << "/" << name << "_" << frames_cnt << ".txt";
		pose_stream >> pose_file;
		ofstream fout(pose_file);
		fout << T;
	}

	void save_octomap_log_to_disk(int voxels, double entropy, string cd, string name,int iterations)
	{
		std::stringstream log_stream, path_stream;
		std::string log_file, path;
		path_stream << "../data" << "_" << process_cnt << cd;
		path_stream >> path;
		access_directory(path);
		log_stream << "../data" << "_" << process_cnt << cd << "/" << name << "_" << iterations << ".txt";
		log_stream >> log_file;
		ofstream fout(log_file);
		fout << voxels << " " << entropy << endl;
	}

	void save_cloud_to_disk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string cd, string name)
	{   //存放点云数据至磁盘，速度很慢，少用
		std::stringstream cloud_stream, path_stream;
		std::string cloud_file, path;
		path_stream << save_path << cd;
		path_stream >> path;
		access_directory(path);
		cloud_stream << save_path << cd << "/" << name << ".pcd";
		cloud_stream >> cloud_file;
		pcl::io::savePCDFile<pcl::PointXYZRGB>(cloud_file, *cloud);
	}

	void save_cloud_to_disk(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string cd, string name, int frames_cnt)
	{   //存放点云数据至磁盘，速度很慢，少用
		std::stringstream cloud_stream, path_stream;
		std::string cloud_file, path;
		path_stream << "../data" << "_" << process_cnt << cd;
		path_stream >> path;
		access_directory(path);
		cloud_stream << "../data" << "_" << process_cnt << cd << "/" << name << "_" << frames_cnt << ".pcd";
		cloud_stream >> cloud_file;
		pcl::io::savePCDFile<pcl::PointXYZRGB>(cloud_file, *cloud);
	}

	void save_octomap_to_disk(octomap::ColorOcTree* octo_model, string cd, string name)
	{   //存放点云数据至磁盘，速度很慢，少用
		std::stringstream octomap_stream, path_stream;
		std::string octomap_file, path;
		path_stream << "../data" << "_" << process_cnt << cd;
		path_stream >> path;
		access_directory(path);
		octomap_stream << "../data" << "_" << process_cnt << cd << "/" << name << ".ot";
		octomap_stream >> octomap_file;
		octo_model->write(octomap_file);
	}

};

inline double pow2(double x) {
	return x * x;
}