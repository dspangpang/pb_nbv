#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "nerf_prv_core/Share_Data.hpp"

inline double pow2(double x) {
  return x * x;
}

class View {
public:
  int id;
  Eigen::Vector3d init_pos;
  Eigen::Matrix4d pose;
  double robot_cost;

  explicit View(const Eigen::Vector3d &_init_pos)
      : id(-1), init_pos(_init_pos), pose(Eigen::Matrix4d::Identity()), robot_cost(0.0) {}

  void get_next_camera_pos(Eigen::Matrix4d now_camera_pose_world,
                           Eigen::Vector3d object_center_world,
                           int type_of_pose = 1) {
    switch (type_of_pose) {
      case 0:
      case 1: {
        Eigen::Vector4d object_center_now_camera =
            now_camera_pose_world.inverse() *
            Eigen::Vector4d(object_center_world(0), object_center_world(1), object_center_world(2), 1);
        Eigen::Vector4d view_now_camera =
            now_camera_pose_world.inverse() *
            Eigen::Vector4d(init_pos(0), init_pos(1), init_pos(2), 1);

        Eigen::Vector3d object(object_center_now_camera(0), object_center_now_camera(1),
                               object_center_now_camera(2));
        Eigen::Vector3d view(view_now_camera(0), view_now_camera(1), view_now_camera(2));
        Eigen::Vector3d z_axis = (object - view).normalized();

        Eigen::Vector3d reference = view.normalized();
        if (reference.norm() < 1e-6 || std::abs(z_axis.dot(reference)) > 0.99) {
          reference = Eigen::Vector3d(0, 0, 1);
        }

        Eigen::Vector3d x_axis = z_axis.cross(reference);
        if (x_axis.norm() < 1e-6) {
          x_axis = z_axis.cross(Eigen::Vector3d(0, 1, 0));
        }
        x_axis.normalize();
        Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T(0, 3) = -view(0);
        T(1, 3) = -view(1);
        T(2, 3) = -view(2);

        Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
        R.block<3, 1>(0, 0) = x_axis;
        R.block<3, 1>(0, 1) = y_axis;
        R.block<3, 1>(0, 2) = z_axis;

        pose = (R.inverse() * T).eval();
        break;
      }
      default:
        throw std::runtime_error("Unsupported pose type.");
    }
  }
};

inline std::pair<int, double> get_local_path(const Eigen::Vector3d &M,
                                             const Eigen::Vector3d &N,
                                             const Eigen::Vector3d &O,
                                             double r) {
  double x0 = O(0), y0 = O(1), z0 = O(2);
  double x1 = M(0), y1 = M(1), z1 = M(2);
  double x2 = N(0), y2 = N(1), z2 = N(2);

  double a = pow2(x2 - x1) + pow2(y2 - y1) + pow2(z2 - z1);
  double b = 2.0 * ((x2 - x1) * (x1 - x0) + (y2 - y1) * (y1 - y0) + (z2 - z1) * (z1 - z0));
  double c = pow2(x1 - x0) + pow2(y1 - y0) + pow2(z1 - z0) - pow2(r);
  double delta = pow2(b) - 4.0 * a * c;

  if (delta <= 0) {
    return std::make_pair(0, (N - M).norm());
  }

  double t3 = (-b - std::sqrt(delta)) / (2.0 * a);
  double t4 = (-b + std::sqrt(delta)) / (2.0 * a);

  if ((t3 < 0 || t3 > 1) && (t4 < 0 || t4 > 1)) {
    return std::make_pair(0, (N - M).norm());
  }
  if ((t3 < 0 || t3 > 1) || (t4 < 0 || t4 > 1)) {
    return std::make_pair(-1, 1e10);
  }
  return std::make_pair(1, (N - M).norm());
}

class CoveragePlanner {
public:
  explicit CoveragePlanner(const Share_Data &_share_data) : share_data(_share_data) {}

  std::vector<View> load_views(int budget) const {
    std::ifstream fin(share_data.viewspace_path + "/" + std::to_string(budget) + ".txt");
    if (!fin.is_open()) {
      throw std::runtime_error("Cannot open hemisphere file for budget " + std::to_string(budget));
    }

    std::vector<Eigen::Vector3d> points;
    double x, y, z;
    while (fin >> x >> y >> z) {
      points.emplace_back(x, y, z);
    }
    fin.close();

    if (points.empty()) {
      throw std::runtime_error("Hemisphere file is empty.");
    }

    double norm = points.front().norm();
    if (norm < 1e-6) {
      norm = 1.0;
    }
    const double scale = share_data.view_space_radius / norm;

    std::vector<View> views;
    views.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
      View view(share_data.object_center_world + points[i] * scale);
      view.id = static_cast<int>(i);
      view.get_next_camera_pos(Eigen::Matrix4d::Identity(), share_data.object_center_world, 1);
      views.push_back(view);
    }
    return views;
  }

  std::vector<int> plan_greedy(const std::vector<View> &views) const {
    std::vector<int> remaining;
    remaining.reserve(views.size());
    for (size_t i = 0; i < views.size(); ++i) {
      remaining.push_back(static_cast<int>(i));
    }

    std::vector<int> ordered_ids;
    ordered_ids.reserve(views.size());
    Eigen::Vector3d current = share_data.start_position;

    while (!remaining.empty()) {
      auto best_it = remaining.begin();
      double best_cost = std::numeric_limits<double>::max();

      for (auto it = remaining.begin(); it != remaining.end(); ++it) {
        const View &candidate = views[*it];
        auto path = get_local_path(current, candidate.init_pos, share_data.object_center_world,
                                   share_data.object_radius);
        double cost = path.second;
        if (cost < best_cost) {
          best_cost = cost;
          best_it = it;
        }
      }

      const int best_id = *best_it;
      ordered_ids.push_back(best_id);
      current = views[best_id].init_pos;
      remaining.erase(best_it);
    }

    return ordered_ids;
  }

  void write_plan(const std::vector<View> &views, const std::vector<int> &ordered_ids) const {
    std::ofstream pose_out(share_data.planned_pose_file_path());
    std::ofstream ids_out(share_data.planned_view_ids_file_path());
    std::ofstream budget_out(share_data.used_budget_file_path());
    if (!pose_out.is_open() || !ids_out.is_open() || !budget_out.is_open()) {
      throw std::runtime_error("Cannot write nerf_prv planner outputs.");
    }

    for (int id : ordered_ids) {
      const View &view = views[id];
      Eigen::Matrix4d world_pose = view.pose.inverse();
      Eigen::Quaterniond q(world_pose.block<3, 3>(0, 0));
      pose_out << world_pose(0, 3) << ' ' << world_pose(1, 3) << ' ' << world_pose(2, 3) << ' '
               << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      ids_out << id << '\n';
    }

    budget_out << ordered_ids.size() << '\n';
    pose_out.close();
    ids_out.close();
    budget_out.close();

    std::ofstream ready_out(share_data.planner_ready_file_path());
    ready_out << "ready\n";
    ready_out.close();
  }

private:
  Share_Data share_data;
};
