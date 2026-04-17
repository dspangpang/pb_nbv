#pragma once

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class Share_Data {
public:
  std::string yaml_file_path;
  std::string work_dir;
  std::string name_of_pcd;
  std::string viewspace_path;
  std::string cache_root;
  std::string prvnet_data_dir;
  std::string external_root;
  std::string upstream_reference_root;
  std::string prvnet_checkpoint;
  std::string prvnet_env_path;
  std::string prvnet_imagenet_path;
  std::string prvnet_dataset_path;
  std::string shape_net_path;
  std::string instant_ngp_scripts_path;
  std::string origin_exports_path;

  bool use_prvnet_prediction;
  int fallback_view_budget;
  int initial_view_budget;
  int max_wait_seconds;

  double view_space_radius;
  double object_radius;
  Eigen::Vector3d object_center_world;
  Eigen::Vector3d start_position;

  std::string save_path;

  explicit Share_Data(const std::string &config_file_path) {
    yaml_file_path = config_file_path;

    const char *work_dir_env = std::getenv("WORK_DIR");
    if (work_dir_env != nullptr) {
      work_dir = std::string(work_dir_env);
    }

    cv::FileStorage fs;
    fs.open(yaml_file_path, cv::FileStorage::READ);
    fs["name_of_pcd"] >> name_of_pcd;
    fs["viewspace_path"] >> viewspace_path;
    fs["cache_root"] >> cache_root;
    fs["prvnet_data_dir"] >> prvnet_data_dir;
    fs["external_root"] >> external_root;
    fs["upstream_reference_root"] >> upstream_reference_root;
    fs["prvnet_checkpoint"] >> prvnet_checkpoint;
    fs["prvnet_env_path"] >> prvnet_env_path;
    fs["prvnet_imagenet_path"] >> prvnet_imagenet_path;
    fs["prvnet_dataset_path"] >> prvnet_dataset_path;
    fs["shape_net_path"] >> shape_net_path;
    fs["instant_ngp_scripts_path"] >> instant_ngp_scripts_path;
    fs["origin_exports_path"] >> origin_exports_path;
    fs["use_prvnet_prediction"] >> use_prvnet_prediction;
    fs["fallback_view_budget"] >> fallback_view_budget;
    fs["initial_view_budget"] >> initial_view_budget;
    fs["max_wait_seconds"] >> max_wait_seconds;
    fs["view_space_radius"] >> view_space_radius;
    fs["object_radius"] >> object_radius;
    fs["object_center_x"] >> object_center_world(0);
    fs["object_center_y"] >> object_center_world(1);
    fs["object_center_z"] >> object_center_world(2);
    fs["start_x"] >> start_position(0);
    fs["start_y"] >> start_position(1);
    fs["start_z"] >> start_position(2);
    fs.release();

    if (viewspace_path.empty() && !work_dir.empty()) {
      viewspace_path = work_dir + "src/nerf_prv_core/view_space/Hemisphere";
    }
    if (cache_root.empty() && !work_dir.empty()) {
      cache_root = work_dir + "src/nerf_prv_core/cache";
    }
    if (prvnet_data_dir.empty() && !work_dir.empty()) {
      prvnet_data_dir = work_dir + "src/nerf_prv_core/prvnet/data";
    }
    if (external_root.empty() && !work_dir.empty()) {
      external_root = work_dir + "src/nerf_prv_core/external";
    }
    if (upstream_reference_root.empty() && !work_dir.empty()) {
      upstream_reference_root = work_dir + "src/nerf_prv_core/upstream/PRV_simulation_reference";
    }
    if (prvnet_checkpoint.empty() && !external_root.empty()) {
      prvnet_checkpoint = external_root + "/prvnet/checkpoints/best_checkpoint.pth";
    }
    if (prvnet_env_path.empty() && !external_root.empty()) {
      prvnet_env_path = external_root + "/convnext_v2";
    }
    if (prvnet_imagenet_path.empty() && !external_root.empty()) {
      prvnet_imagenet_path = external_root + "/prvnet/imagenet/convnextv2_tiny_1k_224_ema.pt";
    }
    if (prvnet_dataset_path.empty() && !external_root.empty()) {
      prvnet_dataset_path = external_root + "/prvnet/data_5view";
    }
    if (shape_net_path.empty() && !external_root.empty()) {
      shape_net_path = external_root + "/shapenet";
    }
    if (instant_ngp_scripts_path.empty() && !external_root.empty()) {
      instant_ngp_scripts_path = external_root + "/instant_ngp/scripts";
    }
    if (origin_exports_path.empty() && !external_root.empty()) {
      origin_exports_path = external_root + "/origin_exports";
    }

    save_path = cache_root + "/" + name_of_pcd;
    access_directory(cache_root);
    access_directory(save_path);
    access_directory(prvnet_data_dir);
    access_directory(prvnet_data_dir + "/images");
    access_directory(external_root);
    access_directory(external_root + "/prvnet");
    access_directory(external_root + "/prvnet/checkpoints");
    access_directory(external_root + "/prvnet/imagenet");
    access_directory(external_root + "/prvnet/data_5view");
    access_directory(external_root + "/convnext_v2");
    access_directory(external_root + "/shapenet");
    access_directory(external_root + "/instant_ngp");
    access_directory(external_root + "/instant_ngp/scripts");
    access_directory(external_root + "/origin_exports");
  }

  std::string budget_file_path() const {
    return prvnet_data_dir + "/view_budget.txt";
  }

  std::string planned_pose_file_path() const {
    return save_path + "/planned_poses.txt";
  }

  std::string planned_view_ids_file_path() const {
    return save_path + "/planned_view_ids.txt";
  }

  std::string planner_ready_file_path() const {
    return save_path + "/planner_ready.txt";
  }

  std::string used_budget_file_path() const {
    return save_path + "/used_view_budget.txt";
  }

  void access_directory(const std::string &directory) const {
    if (directory.empty()) {
      return;
    }

    std::string current;
    for (char ch : directory) {
      current += ch;
      if (ch == '/' && !current.empty() && access(current.c_str(), 0) != 0) {
        mkdir(current.c_str(), 0777);
      }
    }
    if (!current.empty() && access(current.c_str(), 0) != 0) {
      mkdir(current.c_str(), 0777);
    }
  }
};
