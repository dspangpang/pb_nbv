import os
import shutil
import subprocess
import time

import cv2
import numpy as np
import open3d as o3d
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, PointCloud2

from add_model_gazebo import delete_model, spawn_model
from add_realsense_gazebo import spawn_realsense
from set_link_state import set_model_state
from utils import (
    base_to_depth,
    matrix_to_quaternion,
    quaternion_to_matrix,
    save_point_cloud,
)


work_dir = os.environ["WORK_DIR"]
model_dir = ["GSO_models"]
config_file = f"{work_dir}src/nerf_prv_core/config/DefaultConfiguration.yaml"

camera_diff_matrix = np.array(
    [
        [0.0, 1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def read_config_value(file_path, key):
    with open(file_path, "r", encoding="utf-8") as cfg:
        for raw_line in cfg:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith(f"{key}:"):
                value = line.split(":", 1)[1].strip()
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                return value
    return None


def update_config_value(file_path, key, value):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    with open(file_path, "w", encoding="utf-8") as f:
        for line in lines:
            if line.strip().startswith(f"{key}:"):
                if isinstance(value, str):
                    f.write(f'{key}: "{value}"\n')
                else:
                    f.write(f"{key}: {value}\n")
            else:
                f.write(line)


def save_ros_image(msg, path):
    channels = 3
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding.lower() == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(path, image)


def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm < 1e-8:
        return vector
    return vector / norm


def look_at_pose(position, target):
    z_axis = normalize(target - position)
    up = np.array([0.0, 0.0, 1.0], dtype=float)
    x_axis = np.cross(z_axis, up)
    if np.linalg.norm(x_axis) < 1e-8:
        up = np.array([0.0, 1.0, 0.0], dtype=float)
        x_axis = np.cross(z_axis, up)
    x_axis = normalize(x_axis)
    y_axis = normalize(np.cross(z_axis, x_axis))

    pose = np.eye(4)
    pose[:3, 0] = x_axis
    pose[:3, 1] = y_axis
    pose[:3, 2] = z_axis
    pose[:3, 3] = position
    return pose


def planner_pose_to_gazebo_pose(camera_pose):
    gazebo_pose = camera_pose @ camera_diff_matrix
    gazebo_pose = gazebo_pose @ np.linalg.inv(base_to_depth)
    return matrix_to_quaternion(gazebo_pose)


def load_hemisphere_points(file_path, radius):
    points = []
    with open(file_path, "r", encoding="utf-8") as fin:
        for raw_line in fin:
            raw_line = raw_line.strip()
            if not raw_line:
                continue
            values = [float(item) for item in raw_line.split()]
            points.append(values)
    pts = np.asarray(points, dtype=float)
    base_norm = np.linalg.norm(pts[0])
    if base_norm < 1e-8:
        base_norm = 1.0
    return pts * (radius / base_norm)


def wait_for_file(path, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if os.path.isfile(path):
            return True
        time.sleep(0.2)
    return False


if __name__ == "__main__":
    subprocess.Popen(["rosnode", "kill", "-a"])
    time.sleep(2)
    subprocess.Popen(["roslaunch", "env_startup", "benchmark_gazebo_env_startup.launch"])
    time.sleep(2)

    rospy.init_node("start_benchmark_nerf_prv")

    res_data = f"{work_dir}src/gazebo_benchmark_env/env_startup/res_data/"
    linear_velocity = [0.0, 0.0, 0.0]
    angular_velocity = [0.0, 0.0, 0.0]

    realsense_model_pose = Pose()
    realsense_model_pose.position.x = 1.0
    realsense_model_pose.position.y = 0.0
    realsense_model_pose.position.z = 0.0
    realsense_model_pose.orientation.w = 1.0
    spawn_realsense(realsense_model_pose)

    fallback_budget = int(read_config_value(config_file, "fallback_view_budget") or 12)
    initial_view_budget = int(read_config_value(config_file, "initial_view_budget") or 5)
    view_space_radius = float(read_config_value(config_file, "view_space_radius") or 0.9)
    prvnet_data_dir = read_config_value(config_file, "prvnet_data_dir")
    cache_root = read_config_value(config_file, "cache_root")

    for model_type in model_dir:
        model_file_path = f"{work_dir}src/gazebo_benchmark_env/env_startup/models/{model_type}/sdf"
        model_files = os.listdir(model_file_path)

        method = "nerf_prv"
        folder_name = os.path.join(res_data, f"{model_type}_{method}")
        os.makedirs(folder_name, exist_ok=True)

        for model_file in model_files:
            model_name = model_file.split(".")[0]
            pcd_name = f"{model_type}/pcd/{model_name}"
            model_folder = os.path.join(folder_name, model_name)
            os.makedirs(model_folder, exist_ok=True)

            if os.path.exists(os.path.join(model_folder, "done.txt")):
                continue

            shutil.rmtree(model_folder, ignore_errors=True)
            os.makedirs(model_folder, exist_ok=True)

            update_config_value(config_file, "name_of_pcd", pcd_name)
            shutil.copy(config_file, model_folder)

            delete_model(model_name)
            model_pose = Pose()
            model_pose.orientation.w = 1.0
            success, message = spawn_model(
                os.path.join(model_file_path, model_file),
                model_name,
                model_pose,
                "world",
            )
            if not success:
                rospy.logerr("Failed to spawn model: %s", message)
                continue

            time.sleep(1)

            image_dir = os.path.join(prvnet_data_dir, "images")
            os.makedirs(image_dir, exist_ok=True)
            for stale_name in os.listdir(image_dir):
                if stale_name.lower().endswith(".png"):
                    os.remove(os.path.join(image_dir, stale_name))
            budget_file = os.path.join(prvnet_data_dir, "view_budget.txt")
            if os.path.exists(budget_file):
                os.remove(budget_file)

            initial_view_file = (
                f"{work_dir}src/nerf_prv_core/view_space/Hemisphere/{initial_view_budget}.txt"
            )
            initial_points = load_hemisphere_points(initial_view_file, view_space_radius)

            for view_id, point in enumerate(initial_points):
                camera_pose = look_at_pose(point, np.zeros(3))
                position, orientation = planner_pose_to_gazebo_pose(camera_pose)
                set_model_state("realsense", position, orientation, linear_velocity, angular_velocity)
                time.sleep(0.5)

                image_msg = rospy.wait_for_message("/d435/color/image_raw", Image)
                rgb_path = os.path.join(image_dir, f"{view_id}.png")
                save_ros_image(image_msg, rgb_path)

                point_cloud_data = rospy.wait_for_message("/d435/depth/color/points", PointCloud2)
                init_folder = os.path.join(model_folder, "initial_views")
                os.makedirs(init_folder, exist_ok=True)
                init_view_folder = os.path.join(init_folder, f"view_{view_id}")
                os.makedirs(init_view_folder, exist_ok=True)
                save_point_cloud(point_cloud_data, camera_pose, init_view_folder, 0.001)

            subprocess.run(
                ["python3", f"{work_dir}src/nerf_prv_core/prvnet/infer_once.py", config_file],
                check=True,
            )

            used_cache_dir = os.path.join(cache_root, pcd_name)
            planned_pose_file = os.path.join(used_cache_dir, "planned_poses.txt")
            planner_ready_file = os.path.join(used_cache_dir, "planner_ready.txt")
            planned_view_ids_file = os.path.join(used_cache_dir, "planned_view_ids.txt")
            for stale_path in (planned_pose_file, planner_ready_file, planned_view_ids_file):
                if os.path.exists(stale_path):
                    os.remove(stale_path)

            nerf_process = subprocess.Popen(["roslaunch", "nerf_prv_core", "run_nerf_prv.launch"])

            if not wait_for_file(planned_pose_file, 30):
                nerf_process.terminate()
                raise RuntimeError(f"planned_poses.txt not generated for {pcd_name}")

            planned_poses = []
            with open(planned_pose_file, "r", encoding="utf-8") as fin:
                for raw_line in fin:
                    values = [float(item) for item in raw_line.strip().split()]
                    if len(values) == 7:
                        planned_poses.append(values)

            for i, values in enumerate(planned_poses):
                camera_pose = quaternion_to_matrix(values[:3], values[3:])
                position, orientation = planner_pose_to_gazebo_pose(camera_pose)
                iter_folder = os.path.join(model_folder, f"iter_{i + 1}")
                os.makedirs(iter_folder, exist_ok=True)

                with open(os.path.join(iter_folder, "nerf_prv_pose.txt"), "w", encoding="utf-8") as f:
                    f.write(" ".join(str(v) for v in values))

                with open(os.path.join(iter_folder, "gazebo_pose.txt"), "w", encoding="utf-8") as f:
                    f.write(f"{position} {orientation}")

                set_model_state("realsense", position, orientation, linear_velocity, angular_velocity)
                time.sleep(0.5)

                image_msg = rospy.wait_for_message("/d435/color/image_raw", Image)
                save_ros_image(image_msg, os.path.join(iter_folder, "rgb.png"))

                point_cloud_data = rospy.wait_for_message("/d435/depth/color/points", PointCloud2)
                save_point_cloud(point_cloud_data, camera_pose, iter_folder, 0.001)

            with open(os.path.join(model_folder, "done.txt"), "w", encoding="utf-8") as f:
                f.write(str(len(planned_poses) if planned_poses else fallback_budget))

            nerf_process.terminate()
            delete_model(model_name)
