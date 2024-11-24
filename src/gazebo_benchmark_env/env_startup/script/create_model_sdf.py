import os
import open3d as o3d
import trimesh
import numpy as np

model_pkg = "stanford_models"
rescale = True

ply_directory = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/{model_pkg}/models"
scale_ply_directory = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/{model_pkg}/scale_models"
dae_directory = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/{model_pkg}/dae"
sdf_directory = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/{model_pkg}/sdf"
pcd_directory = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/{model_pkg}/pcd"

def create_sdf_content(model_name):
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{dae_directory}/{model_name}.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>{dae_directory}/{model_name}.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    return sdf_content

def convert_ply_to_dae(ply_directory, dae_directory):
    if not os.path.exists(dae_directory):
        os.makedirs(dae_directory)
    if not os.path.exists(scale_ply_directory):
        os.makedirs(scale_ply_directory)

    for filename in os.listdir(ply_directory):
        if filename.endswith(".ply"):
            model_name = os.path.splitext(filename)[0]
            ply_path = os.path.join(ply_directory, filename)
            scale_ply_path = os.path.join(scale_ply_directory, model_name + "_scale.ply")
            dae_path = os.path.join(dae_directory, model_name + ".dae")

            if rescale:
                # Load the PLY file
                mesh = o3d.io.read_triangle_mesh(scale_ply_path)

                # 获取模型的 bounding box
                bbox = mesh.get_axis_aligned_bounding_box()
                # 等比例缩放模型 直到 bounding box 的最大边长为0.3
                min_length = min(bbox.get_max_bound() - bbox.get_min_bound())
                max_length = max(bbox.get_max_bound() - bbox.get_min_bound())
                # print(f"Max length of bounding box: {min_length}")
                scale = 0.2 / min_length
                mesh.scale(scale, center=bbox.get_center())
                # 重新计算 bounding box
                bbox = mesh.get_axis_aligned_bounding_box()

                # 将模型移动到原点
                mesh.translate(-bbox.get_center())

                # print(f"Max length of bounding box after scaling: {min(bbox.get_max_bound() - bbox.get_min_bound())}")
                # 保存缩放后的模型
                o3d.io.write_triangle_mesh(scale_ply_path, mesh)

                # Load the scaled PLY file
                mesh = o3d.io.read_triangle_mesh(scale_ply_path)

                # Convert to trimesh and save as DAE
                vertices = np.asarray(mesh.vertices)
                faces = np.asarray(mesh.triangles)
                if mesh.has_vertex_colors():
                    colors = np.asarray(mesh.vertex_colors)[:, :3]  # Exclude alpha channel
                    # Convert RGB to grayscale
                    grays = 0.299 * colors[:, 0] + 0.587 * colors[:, 1] + 0.114 * colors[:, 2]
                    grays = (grays * 255).astype(np.uint8)  # Convert to 0-255 range
                    # gray gain
                    grays = grays * 2
                    grays = np.stack((grays, grays, grays), axis=-1)  # Duplicate grayscale values to RGB format
                else:
                    print(f"Warning: {filename} does not have vertex colors.")
                    grays = np.full((vertices.shape[0], 3), 128, dtype=np.uint8)  # Default gray color
                
                trimesh_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_colors=grays)
                trimesh_mesh.export(dae_path)

                print(f"Converted {filename} to {model_name}.dae")
            else:
              # 判断 .ply .scale_ply .dae .pcd 文件是否存在
              if os.path.exists(scale_ply_path) and os.path.exists(dae_path):
                  print(f"Model {model_name}.dae already exists.")
                  continue

              # Load the PLY file
              mesh = o3d.io.read_triangle_mesh(ply_path)

              # 如果模型文件的大小超过100MB, 对其进行降采样
              if os.path.getsize(ply_path) > 100000000:
                  print(f"Model {filename} is too large. Simplifying...")
                  mesh = mesh.simplify_quadric_decimation(3000000)
                  print(f"Model {filename} simplified.")

              # 获取模型的 bounding box
              bbox = mesh.get_axis_aligned_bounding_box()
              # 等比例缩放模型 直到 bounding box 的最大边长为0.3
              min_length = min(bbox.get_max_bound() - bbox.get_min_bound())
              # print(f"Max length of bounding box: {min_length}")
              scale = 0.3 / min_length
              mesh.scale(scale, center=bbox.get_center())
              # 重新计算 bounding box
              bbox = mesh.get_axis_aligned_bounding_box()

              # 将模型移动到原点
              mesh.translate(-bbox.get_center())

              # print(f"Max length of bounding box after scaling: {min(bbox.get_max_bound() - bbox.get_min_bound())}")
              # 保存缩放后的模型
              o3d.io.write_triangle_mesh(scale_ply_path, mesh)

              # Load the scaled PLY file
              mesh = o3d.io.read_triangle_mesh(scale_ply_path)

              # Convert to trimesh and save as DAE
              vertices = np.asarray(mesh.vertices)
              faces = np.asarray(mesh.triangles)
              if mesh.has_vertex_colors():
                  colors = np.asarray(mesh.vertex_colors)[:, :3]  # Exclude alpha channel
                  # Convert RGB to grayscale
                  grays = 0.299 * colors[:, 0] + 0.587 * colors[:, 1] + 0.114 * colors[:, 2]
                  grays = (grays * 255).astype(np.uint8)  # Convert to 0-255 range
                  # gray gain
                  grays = grays * 2
                  grays = np.stack((grays, grays, grays), axis=-1)  # Duplicate grayscale values to RGB format
              else:
                  print(f"Warning: {filename} does not have vertex colors.")
                  grays = np.full((vertices.shape[0], 3), 128, dtype=np.uint8)  # Default gray color
              
              trimesh_mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_colors=grays)
              trimesh_mesh.export(dae_path)

              print(f"Converted {filename} to {model_name}.dae")

def convert_ply_to_sdf(ply_directory, sdf_directory):
    if not os.path.exists(sdf_directory):
        os.makedirs(sdf_directory)

    for filename in os.listdir(ply_directory):
        if filename.endswith(".ply"):
            model_name = os.path.splitext(filename)[0]
            sdf_path = os.path.join(sdf_directory, model_name + ".sdf")

            # 判断 .sdf 文件是否存在
            if os.path.exists(sdf_path):
                print(f"Model {model_name}.sdf already exists.")
                continue

            # Create SDF content
            sdf_content = create_sdf_content(model_name)

            # Write SDF file
            with open(sdf_path, "w") as sdf_file:
                sdf_file.write(sdf_content)

            print(f"Created {model_name}.sdf")

def convert_ply_to_pcd(ply_directory, pcd_directory):
    if not os.path.exists(pcd_directory):
        os.makedirs(pcd_directory)

    for filename in os.listdir(ply_directory):
        if filename.endswith(".ply"):
            model_name = os.path.splitext(filename)[0]
            pcd_path = os.path.join(pcd_directory, model_name + ".pcd")
            scale_ply_path = os.path.join(scale_ply_directory, model_name + "_scale.ply")

            if rescale:

                # Load the PLY file
                mesh = o3d.io.read_triangle_mesh(scale_ply_path)

                # Convert to point cloud
                pcd = mesh.sample_points_uniformly(number_of_points=100000)
                o3d.io.write_point_cloud(pcd_path, pcd)

                print(f"Converted {filename} to {model_name}.pcd")
            else :
                # 判断 .pcd 文件是否存在
                if os.path.exists(pcd_path):
                    print(f"Model {model_name}.pcd already exists.")
                    continue

                # Load the PLY file
                mesh = o3d.io.read_triangle_mesh(scale_ply_path)

                # Convert to point cloud
                pcd = mesh.sample_points_uniformly(number_of_points=100000)
                o3d.io.write_point_cloud(pcd_path, pcd)

                print(f"Converted {filename} to {model_name}.pcd")

if __name__ == "__main__":
    convert_ply_to_dae(ply_directory, dae_directory)
    convert_ply_to_sdf(ply_directory, sdf_directory)
    convert_ply_to_pcd(ply_directory, pcd_directory)