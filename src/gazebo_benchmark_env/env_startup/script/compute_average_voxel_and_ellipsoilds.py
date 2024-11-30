import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np
import open3d as o3d
import math
from utils import nbv_iter
import sys

data_dir = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/res_data/comparison"


def process_data(path, group):
    # 获取当前文件夹下的所有文件夹
    folders = os.listdir(path)
    # 提取文件夹名字中带有 '_' 的文件夹
    folders = [folder for folder in folders if '_' in folder]
    folders = [folder for folder in folders if group in folder]
    voxel_total = []
    ellipsoid_total = []

    # 遍历 folders 中的每一个文件夹
    for folder in folders:
        print("Processing folder: ", folder)
        model_type = folder.split('_')[0] + '_' + folder.split('_')[1]
        obj_folders = os.listdir(os.path.join(path, folder))
        obj_folders = [obj_folder for obj_folder in obj_folders if '_' in obj_folder]

        for obj_folder in obj_folders:

            # 如果文件夹下面不存在 done.txt 文件则跳过
            if not os.path.exists(os.path.join(path, folder, obj_folder, 'done.txt')):
                continue

            print("Processing obj folder: ", obj_folder)

            iter_folders = os.listdir(os.path.join(path, folder, obj_folder))
            iter_folders = [iter_folder for iter_folder in iter_folders if '_' in iter_folder]

            # 剔除所有带 . 的文件夹
            iter_folders = [iter_folder for iter_folder in iter_folders if '.' not in iter_folder]

            # 排序 iter_folders
            iter_folders.sort(key=lambda x: int(x.split('_')[-1]))

            # 清除 iter_folders 里面是没有 point_cloud.pcd 的文件夹
            for iter_folder in iter_folders:
                if not os.path.exists(os.path.join(path, folder, obj_folder, iter_folder, 'point_cloud.pcd')):
                    iter_folders.remove(iter_folder)

            for i in range(len(iter_folders)):

                with open(os.path.join(path, folder, obj_folder, iter_folders[i], 'ellipsoid_num.txt'), 'r') as f:
                    # 读取文件内容
                    lines = f.readlines()
                    # 遍历每一行
                    for line in lines:
                        # 提取时间
                        ellipsoid_total.append(int(line))
                
                with open(os.path.join(path, folder, obj_folder, iter_folders[i], 'voxel_num.txt'), 'r') as f:
                    # 读取文件内容
                    lines = f.readlines()
                    # 遍历每一行
                    for line in lines:
                        # 提取时间
                        voxel_total.append(int(line))


    # 新建一个 txt 文件 保存平均 ellipsoid_num
    average_ellipsoid = sum(ellipsoid_total) / len(ellipsoid_total)
    with open(os.path.join(path, "figuredata", f"{group}_time_average.txt"), 'w') as f:
        f.write(str(average_ellipsoid))

    # 新建一个 txt 文件 保存平均 voxel_num
    average_voxel = sum(voxel_total) / len(voxel_total)
    with open(os.path.join(path, "figuredata", f"{group}_time_iter_average.txt"), 'w') as f:
        f.write(str(average_voxel))

if __name__ == '__main__':
            
    print("Reading data...")


    process_data(data_dir, group)






