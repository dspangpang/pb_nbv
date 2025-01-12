import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np
import open3d as o3d
import math
from utils import nbv_iter
import sys

# 从环境变量中获取工作目录
work_dir = os.environ['WORK_DIR']

data_dir = f"{work_dir}src/gazebo_benchmark_env/env_startup/res_data"
model_dir = f"{work_dir}src/gazebo_benchmark_env/env_startup/models"

print("Data dir: ", data_dir)
print("Model dir: ", model_dir)
print("nbv iter: ", nbv_iter)

def draw_frame(time, coverage, model_name, path):

    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'
    
    # 确保字体文件被正确加载
    font_prop = font_manager.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    
    fig, ax1 = plt.subplots()
    # 添加网格
    ax1.grid(True)
    # 设置y轴范围
    ax1.set_ylim(0, 1)
    ax1.plot(range(len(coverage)), coverage ,marker='s', linestyle = '-.', lw=2, label='Coverage')
    ax1.set_ylabel('Coverage(%)', color='black')
    ax1.tick_params(axis='y', labelcolor='black')

    ax1.set_xticks(np.arange(len(coverage)))
    ax1.set_xticklabels(np.arange(1, len(coverage)+1))

    # 创建第二个 y 轴
    ax2 = ax1.twinx()
    ax2.bar(range(len(time)), time, color='b', alpha=0.6, label='Time')
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Time(s)', color='black')
    ax2.set_ylim(0, max(time)*2)
    ax2.tick_params(axis='y', labelcolor='black')

    # 设置 x 轴标签
    ax1.set_xlabel('Iteration')

    # 设置 x 轴刻度 从 1 开始
    plt.xticks(np.arange(0, len(time), 1))

    # 添加图例
    fig.legend(loc='upper left', bbox_to_anchor=(0.1, 0.9))

    # 添加标题
    plt.title(model_name)

    # 保存图片
    # 如果文件存在则删除
    if os.path.exists(os.path.join(path, f"{model_name}.svg")):
        os.remove(os.path.join(path, f"{model_name}.svg"))

    plt.savefig(os.path.join(path, f"{model_name}.svg"))
    plt.close(fig)  # 关闭图形

def draw_compare_converga_frame(groups, path):
    # 读取数据
    coverage_group = []
    for group in groups:
        with open(os.path.join(path, "figuredata", f"{group}_coverage_average.txt"), 'r') as f:
            lines = f.readlines()
            coverage = [float(line) for line in lines]
            coverage_group.append(coverage)
    
    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'
    
    # 确保字体文件被正确加载
    font_prop = font_manager.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    
    # 绘制4个折线图
    fig, ax1 = plt.subplots()
    # 添加网格
    ax1.grid(True)
    # 设置y轴范围
    ax1.set_ylim(0, 110)
    ax1.set_xlabel('Iteration')
    ax1.set_xticks(np.arange(len(coverage_group[0])))
    ax1.set_xticklabels(np.arange(1, len(coverage_group[0])+1))
    for i in range(len(groups)):
        coverage = np.array(coverage_group[i]) * 100.0  # 确保 coverage 是一个 NumPy 数组
        if groups[i] == 'mcmf':
            label = 'MCMF'
            line_a_1 = ax1.plot(coverage, label=label, marker='s', linestyle='-.', lw=2, color='slateblue')
            continue
        if groups[i] == 'scvp':
            label = 'SCVP'
            line_a_2 = ax1.plot(coverage, label=label, marker='o', linestyle='-.', lw=2)
            continue
        if groups[i] == 'nbvnet':
            label = 'NBVNET'
            line_a_3 = ax1.plot(coverage, label=label, marker='^', linestyle='dotted', lw=2)
            continue
        if groups[i] == 'see':
            label = 'SEE'
            line_a_4 = ax1.plot(coverage, label=label, marker='x', linestyle='dotted', lw=2)
            continue
        else:
            label = "Ours"
            line_a_5 = ax1.plot(coverage, label=label, marker='s', linestyle='dotted', lw=2, color="red")

    ax1.set_ylabel('Coverage (%)', color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    
    # 添加图例
    fig.legend(loc='upper left', bbox_to_anchor=(0.12, 0.88))

    # 保存图片
    plt.savefig(os.path.join(path, "figuredata", "compare_coverage.svg"))
    plt.close(fig)  # 关闭图形

def draw_compare_time_frame(groups, path):
    # 读取数据
    time_group = []
    for group in groups:
        with open(os.path.join(path, "figuredata", f"{group}_time_average.txt"), 'r') as f:
            lines = f.readlines()
            coverage = [float(line) for line in lines]
            time_group.append(coverage)
    
    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'
    
    # 确保字体文件被正确加载
    font_prop = font_manager.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    
    # 绘制4个折线图
    fig, ax1 = plt.subplots()
    # 添加网格
    ax1.grid(True)
    # 设置y轴范围
    # 获取 time_group 中的最大值
    max_time = 0
    for i in range(len(time_group)):
        max_time = max(max_time, max(time_group[i])*1.1)
    ax1.set_ylim(0, max_time)

    ax1.set_xlabel('Iteration')
    ax1.set_xticks(np.arange(len(time_group[0])))
    ax1.set_xticklabels(np.arange(1, len(time_group[0])+1))
    for i in range(len(groups)):
        coverage = np.array(time_group[i])
        if groups[i] == 'mcmf':
            label = 'MCMF'
            line_a_1 = ax1.plot(coverage, label=label, marker='s', linestyle='-.', lw=2, color='slateblue')
            continue
        if groups[i] == 'scvp':
            label = 'SCVP'
            line_a_2 = ax1.plot(coverage, label=label, marker='o', linestyle='-.', lw=2)
            continue
        if groups[i] == 'nbvnet':
            label = 'NBVNET'
            line_a_3 = ax1.plot(coverage, label=label, marker='^', linestyle='dotted', lw=2)
            continue
        if groups[i] == 'see':
            label = 'SEE'
            line_a_4 = ax1.plot(coverage, label=label, marker='x', linestyle='dotted', lw=2)
            continue
        else:
            label = "Ours"
            line_a_5 = ax1.plot(coverage, label=label, marker='s', linestyle='dotted', lw=2, color="red")

    ax1.set_ylabel('Time (s)', color='black')
    ax1.tick_params(axis='y', labelcolor='black')
    
    # 添加图例
    fig.legend(loc='upper left', bbox_to_anchor=(0.12, 0.88))

    # 保存图片
    plt.savefig(os.path.join(path, "figuredata", "compare_times.svg"))
    plt.close(fig)  # 关闭图形


def read_groups(path):
    # 获取当前文件夹下的所有文件夹
    folders = os.listdir(path)
    # 提取文件夹名字中带有 '_' 的文件夹
    folders = [folder for folder in folders if '_' in folder]

    # 在文件夹名字中找到最后一个 '_' 的位置
    # 以此为分隔符，分割文件夹名字
    # 保留分隔符之后的字符串
    groups = [folder.split('_')[-1] for folder in folders]
    # 去重
    groups = list(set(groups))
    print("Groups: ", groups)
    return groups

def process_data(path, group):
    # 获取当前文件夹下的所有文件夹
    folders = os.listdir(path)
    # 提取文件夹名字中带有 '_' 的文件夹
    folders = [folder for folder in folders if '_' in folder]
    folders = [folder for folder in folders if group in folder]
    time_iter_average = [0] * nbv_iter
    coverage_iter_average = [0] * nbv_iter
    time_average = 0
    time_total = []
    cnt_per_iter = [0] * nbv_iter
    model_num = 0

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
            
            # 用于计算 平均覆盖率
            model_num += 1

            print("Processing obj folder: ", obj_folder)
            model_name = obj_folder
            full_model_name = model_type + '_' + model_name
            model_path = os.path.join(model_dir, model_type, "pcd", f"{model_name}.pcd")

            # 用于计算点云覆盖率
            ground_truth = o3d.io.read_point_cloud(model_path)
            current_data = o3d.geometry.PointCloud()

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

            time_current = [0] * len(iter_folders)
            coverage_current = [0] * len(iter_folders)

            for i in range(len(iter_folders)):
                # 读取文件夹下的 time.txt 文件
                with open(os.path.join(path, folder, obj_folder, iter_folders[i], 'time.txt'), 'r') as f:
                    # 读取文件内容
                    lines = f.readlines()
                    # 遍历每一行
                    for line in lines:
                        # 提取时间
                        time = float(line)
                        time_iter_average[i] += time
                        time_current[i] = time
                        time_total.append(time)

                # 读取文件夹下的 point_cloud.pcd 文件
                current_data += o3d.io.read_point_cloud(os.path.join(path, folder, obj_folder, iter_folders[i], 'point_cloud.pcd'))

                # 遍历所有 model_pcd 的点 在 input_pcd_tree 中找最近点 计算重叠率
                result = 0.0
                distance_threshold = 0.005
                current_data_tree = o3d.geometry.KDTreeFlann(current_data)
                for j in range(len(ground_truth.points)):
                    [k, idx, dis] = current_data_tree.search_knn_vector_3d(ground_truth.points[j], 1)
                    if math.sqrt(dis[0]) < distance_threshold:
                        result += 1

                coverage = result / len(ground_truth.points)
                coverage_iter_average[i] += coverage
                coverage_current[i] = coverage
                cnt_per_iter[i] += 1

            # 如果 coverage_current 的长度小于 nbv_iter 则用最后一位补全                                                                     
            if len(coverage_current) < nbv_iter:
                for i in range(nbv_iter - len(coverage_current)):
                    coverage_current.append(coverage_current[-1])
                    coverage_iter_average[len(coverage_current)-1] += coverage_current[-1]
                    

            draw_frame(time_current, coverage_current, full_model_name+group, os.path.join(path, folder, obj_folder))

    # 计算平均时间
    for i in range(nbv_iter):
        time_iter_average[i] /= cnt_per_iter[i]
        coverage_iter_average[i] /= model_num

    draw_frame(time_iter_average, coverage_iter_average, group, os.path.join(path, "figuredata"))

    # 新建一个 txt 文件 保存平均时间
    with open(os.path.join(path, "figuredata", f"{group}_time_average.txt"), 'w') as f:
        for time in time_iter_average:
            f.write(str(time) + '\n')

    # 新建一个 txt 文件 保存平均覆盖率
    with open(os.path.join(path, "figuredata", f"{group}_coverage_average.txt"), 'w') as f:
        for coverage in coverage_iter_average:
            f.write(str(coverage) + '\n')

    # 新建一个 txt 文件 保存总时间
    with open(os.path.join(path, "figuredata", f"{group}_time_total.txt"), 'w') as f:
        for time in time_total:
            f.write(str(time) + '\n')

    return time_average, time_iter_average

if __name__ == '__main__':
            
    print("Reading data...")

    # 判断程序是否有输入参数
    if len(sys.argv) < 2:
        print("Please input the group name")
        groups = read_groups(data_dir)
        time_average_group = []
        time_iter_average_group = []

        for group in groups:

            # 如果 group.svg 存在则跳过
            if os.path.exists(os.path.join(data_dir, "figuredata", f"{group}.svg")):
                continue

            time_average, time_iter_average = process_data(data_dir, group)
            time_average_group.append(time_average)
            time_iter_average_group.append(time_iter_average)
    
        draw_compare_converga_frame(groups, data_dir)
        draw_compare_time_frame(groups, data_dir)

    else:
        # 读取程序输入参数
        input_group = str(sys.argv[1])
        print("Input group: ", input_group)
        process_data(data_dir, input_group)




