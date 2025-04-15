import os
import open3d as o3d
import trimesh
import numpy as np

model_pkg = "lm_models"
rescale = False

# 从环境变量中获取工作目录
work_dir = os.environ['WORK_DIR']
pcd_directory = f"{work_dir}src/gazebo_benchmark_env/env_startup/models/{model_pkg}/pcd"
pcd_without_base = f"{work_dir}src/gazebo_benchmark_env/env_startup/models/{model_pkg}/pcd_without_base"
# 创建目录
if not os.path.exists(pcd_without_base):
    os.makedirs(pcd_without_base)

base_thres = 0.1

def delete_model_base():
    for filename in os.listdir(pcd_directory):
        if filename.endswith(".pcd"):
            model_name = os.path.splitext(filename)[0]
            pcd_path = os.path.join(pcd_directory, model_name + ".pcd")
            origin = o3d.io.read_point_cloud(pcd_path)

            # 获取点云的点和颜色
            points = np.asarray(origin.points)
            colors = np.asarray(origin.colors)

            # 检查点云是否为空
            if points.shape[0] == 0:
                print(f"Warning: No points found in {pcd_path}. Skipping.")
                continue

            # 获取点云的最小 z 值
            min_z = np.min(points[:, 2])

            # 创建布尔掩码，保留 z > min_z + base_thres 的点
            mask = points[:, 2] > min_z + base_thres
            filtered_points = points[mask]

            # 如果点云包含颜色，则同步过滤颜色
            if colors.shape[0] == points.shape[0]:
                filtered_colors = colors[mask]
            else:
                filtered_colors = None

            # 创建新的点云对象
            filtered_pcd = o3d.geometry.PointCloud()
            filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
            if filtered_colors is not None:
                filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

            # 保存滤波后的点云
            filtered_pcd_path = os.path.join(pcd_without_base, model_name + ".pcd")
            o3d.io.write_point_cloud(filtered_pcd_path, filtered_pcd)
            print(f"Filtered point cloud saved to {filtered_pcd_path}")

if __name__ == "__main__":
    delete_model_base()
    print("All models processed.")
