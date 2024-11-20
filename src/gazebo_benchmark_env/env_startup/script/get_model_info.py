import os
import json
import numpy as np
import open3d as o3d
from tqdm import tqdm

def compute_bounding_box(cloud):
    bbox = cloud.get_axis_aligned_bounding_box()
    min_pt = bbox.min_bound
    max_pt = bbox.max_bound
    size = max_pt - min_pt
    return min_pt, size

def compute_diameter(cloud):
    bbox = cloud.get_axis_aligned_bounding_box()
    min_pt = bbox.min_bound
    max_pt = bbox.max_bound
    diameter = np.linalg.norm(max_pt - min_pt)
    return diameter

def process_ply_files(directory):
    models_info = {}
    ply_files = [f for f in os.listdir(directory) if f.endswith(".ply")]
    for filename in tqdm(ply_files, desc="Processing PLY files"):
        model_id = os.path.splitext(filename)[0]
        ply_path = os.path.join(directory, filename)
        cloud = o3d.io.read_point_cloud(ply_path)

        min_pt, size = compute_bounding_box(cloud)
        diameter = compute_diameter(cloud)

        models_info[model_id] = {
            "diameter": diameter,
            "min_x": min_pt[0],
            "min_y": min_pt[1],
            "min_z": min_pt[2],
            "size_x": size[0],
            "size_y": size[1],
            "size_z": size[2]
        }

    return models_info

def main():
    directory = "src/gazebo_benchmark_env/env_startup/models/stanford_models"
    models_info = process_ply_files(directory)

    with open("models_info.json", "w") as json_file:
        json.dump(models_info, json_file, indent=2)

if __name__ == "__main__":
    main()