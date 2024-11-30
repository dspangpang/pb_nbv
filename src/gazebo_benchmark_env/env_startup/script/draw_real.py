import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np
import open3d as o3d
import math

current_dir = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/res_data/arealdata"

point_cloud_num_total = []
time_total = []
coverages_total = []
folder_names = []

def read_data():

    global folder_names

    # 获取当前文件夹下的所有文件夹
    folders = os.listdir(current_dir)
    # folders = ['dog', 'rhino', 'statue', 'vase']
    # folders = ['statue', 'rhino', 'vase', 'dog']
    folder_names = folders
    print(folders)
    # 遍历文件夹
    for folder in folders:
        point_cloud_num = np.array([])
        time = np.array([])
        coverages = np.array([])
        # 读取文件夹下的 point_cloud_num.txt 文件
        with open(os.path.join(current_dir, folder, 'time/point_cloud_num.txt'), 'r') as f:
            # 读取文件内容
            lines = f.readlines()
            # 遍历每一行
            for line in lines:
                # 提取覆盖率
                point_cloud_num = np.append(point_cloud_num, float(line))
        # 提取时间
        with open(os.path.join(current_dir, folder, 'time/compute_time.txt'), 'r') as f:
            lines = f.readlines()
            for line in lines:
                time = np.append(time, float(line))

        # 判断是否存在 coverage.txt 文件
        if os.path.exists(os.path.join(current_dir, folder, 'time/coverage.txt')):
            print("Read coverags.txt")
            with open(os.path.join(current_dir, folder, 'time/coverage.txt'), 'r') as f:
                lines = f.readlines()
                for line in lines:
                    coverages = np.append(coverages, float(line))
        else:
            print("Compute coverags: ")
            # 读取 refine 中的点云数据
            point_cloud = o3d.geometry.PointCloud()
            cnt = 0
            for file in os.listdir(os.path.join(current_dir, folder, 'refine')):
                if file.endswith('.pcd'):
                    point_cloud += o3d.io.read_point_cloud(os.path.join(current_dir, folder, 'refine', file))
                    cnt += 1

            current_point_cloud = o3d.geometry.PointCloud()
            
            # 创建一个txt文件，用于存储覆盖率
            with open(os.path.join(current_dir, folder, 'time/coverage.txt'), 'w') as f:
                pass

            for i in range(cnt):
                current_point_cloud += o3d.io.read_point_cloud(os.path.join(current_dir, folder, 'refine', f'refine{i+1}.pcd'))
                result = 0.0
                distance_threshold = 0.005
                current_data_tree = o3d.geometry.KDTreeFlann(current_point_cloud)
                for j in range(len(point_cloud.points)):
                    [k, idx, dis] = current_data_tree.search_knn_vector_3d(point_cloud.points[j], 1)
                    if math.sqrt(dis[0]) < distance_threshold:
                        result += 1

                coverage = result / len(point_cloud.points)
                with open(os.path.join(current_dir, folder, 'time/coverage.txt'), 'a') as f:
                    f.write(str(coverage) + '\n')
                coverages = np.append(coverages, coverage)

        point_cloud_num_total.append(point_cloud_num)
        time_total.append(time)
        coverages_total.append(coverages)
        
    
def draw_data_num():

    global folder_names

    # 遍历所有文件夹 把文件夹名字中'_'之前的字符相同的分为一组    
    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'
    if not os.path.exists(font_path):
        raise FileNotFoundError(f"Font file not found: {font_path}")
        # 确保字体文件被正确加载
    font_prop = font_manager.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    handles, labels = [], []

    # 绘制点云个数和时间为一张图
    # 点云个数为折线图 时间为柱状图
    # 柱状图和折线图共用 x 轴
    # 柱状图分开不是在一个柱子上
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    
    # 计算每个条形图的宽度，这里的0.8是为了在条形图之间留出一些空间
    num_folders = len(folder_names)
    bar_width = 0.8 / num_folders        
    
    for i, folder in enumerate(folder_names):
        index = folder_names.index(folder)

        # 添加网格
        ax1.grid(True)
        # 设置y轴范围
        ax1.set_ylim(-4000, 10000)
        custom_yticks = [0, 2000, 4000, 6000, 8000, 10000]
        custom_yticklabels = ['0', '2000', '4000', '6000', '8000', '10000']
        ax1.set_yticks(custom_yticks)
        ax1.set_yticklabels(custom_yticklabels)
        # 设置x轴坐标的间隔
        ax1.set_xticks(np.arange(len(point_cloud_num_total[index])))
        ax1.set_xticklabels(np.arange(1, len(point_cloud_num_total[index])+1))

        # 计算每个条形图的x位置
        x_positions = np.arange(len(time_total[index])) + (i * bar_width - 0.5 + 0.5 / 2)

        if folder == 'dog':
            label = 'Dog'
            line_a_1 = ax1.plot(point_cloud_num_total[index], label=label, marker='s', linestyle = '-.', lw=2, color='slateblue')
            line_b_1 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label, color='slateblue')
        if folder == 'rhino':
            label = 'Rhino'
            line_a_2 = ax1.plot(point_cloud_num_total[index], label=label, marker='o', linestyle = '-.', lw=2)
            line_b_2 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)
        if folder == 'statue':
            label = 'Statue'
            line_a_3 = ax1.plot(point_cloud_num_total[index], label=label, marker='^', linestyle = 'dotted', lw=2)
            line_b_3 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)
        if folder == 'vase':
            label = 'Vase'
            line_a_4 = ax1.plot(point_cloud_num_total[index], label=label, marker='x', linestyle = 'dotted', lw=2)
            line_b_4 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)

    # 设置y轴标签
    ax1.set_ylabel('Number of Point cloud ')
    # 设置x轴标签
    ax1.set_xlabel('Iteration')
    # ax1.set_title('Point cloud number and calculation time')

    # 设置y轴范围
    ax2.set_ylim(0, 10)
    # 设置y轴标签
    ax2.set_ylabel('Calculation time (s)')
    
    # 显示图例
    fig.legend(loc='upper center', bbox_to_anchor=(0.7, 0.7), ncol=2)
    plt.show()

def draw_data_coverage():
    
    global folder_names

    # 遍历所有文件夹 把文件夹名字中'_'之前的字符相同的分为一组    
    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'
    if not os.path.exists(font_path):
        raise FileNotFoundError(f"Font file not found: {font_path}")
        # 确保字体文件被正确加载
    font_prop = font_manager.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    handles, labels = [], []

    # 绘制点云个数和时间为一张图
    # 点云个数为折线图 时间为柱状图
    # 柱状图和折线图共用 x 轴
    # 柱状图分开不是在一个柱子上
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    
    # 计算每个条形图的宽度，这里的0.8是为了在条形图之间留出一些空间
    num_folders = len(folder_names)
    bar_width = 0.8 / num_folders        
    
    for i, folder in enumerate(folder_names):
        index = folder_names.index(folder)

        # 添加网格
        ax1.grid(True)
        # 设置y轴范围
        ax1.set_ylim(0, 110)
        custom_yticks = [0, 20, 40, 60, 80, 100]
        custom_yticklabels = ['0', '20', '40', '60', '80', '100']
        ax1.set_yticks(custom_yticks)
        ax1.set_yticklabels(custom_yticklabels)
        # 设置x轴坐标的间隔
        ax1.set_xticks(np.arange(len(coverages_total[index])))
        ax1.set_xticklabels(np.arange(1, len(coverages_total[index])+1))

        # 计算每个条形图的x位置
        x_positions = np.arange(len(time_total[index])) + (i * bar_width - 0.5 + 0.5 / 2)

        if folder == 'dog':
            label = 'Dog'
            line_a_1 = ax1.plot(coverages_total[index] * 100, label=label, marker='s', linestyle = '-.', lw=2, color='slateblue')
            line_b_1 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label, color='slateblue')
        if folder == 'rhino':
            label = 'Rhino'
            line_a_2 = ax1.plot(coverages_total[index] * 100, label=label, marker='o', linestyle = '-.', lw=2)
            line_b_2 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)
        if folder == 'statue':
            label = 'Statue'
            line_a_3 = ax1.plot(coverages_total[index] * 100, label=label, marker='^', linestyle = 'dotted', lw=2)
            line_b_3 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)
        if folder == 'vase':
            label = 'Vase'
            line_a_4 = ax1.plot(coverages_total[index] * 100, label=label, marker='x', linestyle = 'dotted', lw=2)
            line_b_4 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)

    # 设置y轴标签
    ax1.set_ylabel('Coverage (%)')
    # 移动 ylabels 的位置
    ax1.yaxis.set_label_coords(-0.08, 0.5)
    # 设置x轴标签
    ax1.set_xlabel('Iteration')
    # ax1.set_title('Point cloud number and calculation time')

    # 设置y轴范围
    ax2.set_ylim(0, 10)
    # 设置y轴标签
    ax2.set_ylabel('Calculation time (s)')
    
    # 显示图例
    fig.legend(loc='upper center', bbox_to_anchor=(0.7, 0.7), ncol=2)
    plt.show()

if __name__ == '__main__':
    print("开始处理数据: ", current_dir)
    read_data()
    print("point_cloud_num_total: ", point_cloud_num_total)
    print("coverages_total: ", coverages_total)
    print("time_total: ", time_total)
    draw_data_coverage()
    print("数据处理完成")
    