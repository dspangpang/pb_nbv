import open3d as o3d
import numpy as np
import os

model_dir = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models"
model_types = ["hb_models", "stanford_models", "lm_models"]
model_size = "scale_models"
model_total_num = 55
def load_and_transform_meshes(h_spacing=0.7, v_spacing=0.5):
    meshes = []
    cnt = 1
    for model_type in model_types:
        model_folder = os.path.join(model_dir, model_type, model_size)
        model_files = os.listdir(model_folder)
        print("Model type: ", model_type)
        print("Model folder: ", model_folder)
        print("Model files num: ", len(model_files))
        for model_file in model_files:
            print("Loading model: ", model_file)
            mesh = o3d.io.read_triangle_mesh(os.path.join(model_folder, model_file))
            # 每行摆7个模型，然后换行
            mesh.translate(np.array([cnt % 7 * h_spacing, -cnt // 7 * v_spacing, 0]))
            meshes.append(mesh)
            cnt += 1
    return meshes

def visualize_meshes(meshes):
    o3d.visualization.draw_geometries(meshes)

if __name__ == "__main__":
    meshes = load_and_transform_meshes()
    visualize_meshes(meshes)