#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <ros/ros.h>

#include "nerf_prv_core/Share_Data.hpp"
#include "nerf_prv_core/View_Space.hpp"

namespace {

int read_budget_with_fallback(const Share_Data &share_data) {
  const std::string budget_file = share_data.budget_file_path();
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(share_data.max_wait_seconds);

  while (std::chrono::steady_clock::now() < deadline) {
    std::ifstream fin(budget_file);
    if (fin.is_open()) {
      int budget = -1;
      fin >> budget;
      fin.close();
      if (budget > 0) {
        ROS_INFO_STREAM("nerf_prv_core: received predicted view budget " << budget);
        return budget;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  ROS_WARN_STREAM("nerf_prv_core: no view_budget.txt after " << share_data.max_wait_seconds
                                                             << " s, using fallback budget "
                                                             << share_data.fallback_view_budget);
  return share_data.fallback_view_budget;
}

std::string default_config_path() {
  const char *work_dir_env = std::getenv("WORK_DIR");
  if (work_dir_env == nullptr) {
    throw std::runtime_error("WORK_DIR is not set.");
  }
  return std::string(work_dir_env) + "src/nerf_prv_core/config/DefaultConfiguration.yaml";
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "nerf_prv_core_node");
  ros::NodeHandle nh("~");

  try {
    std::string config_path;
    nh.param<std::string>("config_path", config_path, default_config_path());

    Share_Data share_data(config_path);
    const int budget = read_budget_with_fallback(share_data);

    CoveragePlanner planner(share_data);
    const std::vector<View> views = planner.load_views(budget);
    const std::vector<int> ordered_ids = planner.plan_greedy(views);
    planner.write_plan(views, ordered_ids);

    ROS_INFO_STREAM("nerf_prv_core: wrote " << ordered_ids.size() << " planned poses to "
                                            << share_data.planned_pose_file_path());
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("nerf_prv_core failed: " << e.what());
    return 1;
  }

  return 0;
}
