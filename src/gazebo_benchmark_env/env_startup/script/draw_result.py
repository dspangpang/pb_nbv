import os
import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np
import open3d as o3d
import math

data_dir = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/res_data"
model_dir = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models"

def read_data():
    global folder_names
    global coverage_total
    global time_total  

    # 获取当前文件夹下的所有文件夹
    folders = os.listdir(current_dir)
    # 提取文件夹名字中带有 '_' 的文件夹
    folders = [folder for folder in folders if '_' in folder]
    # 把 folders 按首字母排序
    folders.sort()
    print(folders)
    folder_names = folders
    # 遍历文件夹
    for folder in folder_names:
        coverage = np.array([])
        time = np.array([])
        # 读取文件夹下的 coverage.txt 文件
        with open(os.path.join(current_dir, folder, 'clouds/coverage.txt'), 'r') as f:
            # 读取文件内容
            lines = f.readlines()
            # 遍历每一行
            for line in lines:
                # 提取覆盖率
                coverage = np.append(coverage, float(line))
                # 提取时间
        with open(os.path.join(current_dir, folder, 'compute_time.txt'), 'r') as f:
            lines = f.readlines()
            for line in lines:
                time = np.append(time, float(line))
        
        coverage_total.append(coverage)
        time_total.append(time)
        
    
def draw_data():
    global folder_names
    global coverage_total
    global time_total  
    
    # 遍历所有文件夹 把文件夹名字中'_'之前的字符相同的分为一组
    groups = {}
    for folder in folder_names:
        key = folder.split('_')[0]
        if key not in groups:
            groups[key] = []
        groups[key].append(folder)
    

    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'
    if not os.path.exists(font_path):
        raise FileNotFoundError(f"Font file not found: {font_path}")
        # 确保字体文件被正确加载
    font_prop = font_manager.FontProperties(fname=font_path)
    plt.rcParams['font.family'] = font_prop.get_name()
    handles, labels = [], []
    # 遍历每一组
    # 绘制覆盖率和时间为一张图
    # 覆盖率为折线图 时间为柱状图
    # 柱状图和折线图共用 x 轴
    # 柱状图分开不是在一个柱子上
    for key in groups:
        fig, ax1 = plt.subplots()
        ax2 = ax1.twinx()
        
        # 计算每个条形图的宽度，这里的0.8是为了在条形图之间留出一些空间
        num_folders = len(groups[key])
        bar_width = 0.8 / num_folders        
        
        for i, folder in enumerate(groups[key]):
            index = folder_names.index(folder)
            label = ''

            # 添加网格
            ax1.grid(True)
            # 设置y轴范围
            ax1.set_ylim(0, 105)
            # 设置x轴坐标的间隔
            ax1.set_xticks(np.arange(len(coverage_total[index])))
            ax1.set_xticklabels(np.arange(1, len(coverage_total[index])+1))

            # 计算每个条形图的x位置
            x_positions = np.arange(len(time_total[index])) + (i * bar_width - 0.5 + 0.5 / 2)

            if folder.split('_')[1] == '0':
                label = 'MFMR'
                line_a_1 = ax1.plot(coverage_total[index]*100, label=label, marker='s', linestyle = '-.', lw=2)
                line_b_1 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)
            if folder.split('_')[1] == '4':
                label = 'APORA'
                line_a_2 = ax1.plot(coverage_total[index]*100, label=label, marker='o', linestyle = '-.', lw=2)
                line_b_2 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)
            if folder.split('_')[1] == 'x':
                label = 'OURS'
                line_a_3 = ax1.plot(coverage_total[index]*100, label=label, marker='^', linestyle = 'dotted', lw=2)
                line_b_3 = ax2.bar(x_positions, time_total[index] / 1000, bar_width, alpha=1, label=label)

            # 设置y轴标签
            ax1.set_ylabel('Coverage rate (%)')
            # 设置x轴标签
            ax1.set_xlabel('Iteration')
            # 设置标题
            if(key == 'Happy'):
                ax1.set_title('Baddha')
            else:
                ax1.set_title(key)

            # 设置y轴范围
            ax2.set_ylim(0, 35)
            # 设置y轴标签
            ax2.set_ylabel('Calculation time (s)')
        
        # 显示图例
        fig.legend(loc='upper center', bbox_to_anchor=(0.7, 0.7), ncol=2)
        plt.show()


def draw_frame(time, coverage, model_name, path):

    plt.rcParams.update({'font.size': 11})
    # 使用 fc-list 查找到的字体路径
    font_path = '/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/config/Times New Roman.ttf'
    
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

    # 创建第二个 y 轴
    ax2 = ax1.twinx()
    ax2.bar(range(len(time)), time, color='b', alpha=0.6, label='Time')
    ax2.set_xlabel('Iteration')
    ax2.set_ylabel('Time(s)', color='black')
    ax2.set_ylim(0, max(time)*2)
    ax2.tick_params(axis='y', labelcolor='black')

    # 添加图例
    fig.legend(loc='upper left', bbox_to_anchor=(0.1, 0.9))

    # 添加标题
    plt.title(model_name)

    # 保存图片
    plt.savefig(os.path.join(path, f"{model_name}.svg"))

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

    return groups

def process_data(path, group):
    # 获取当前文件夹下的所有文件夹
    folders = os.listdir(path)
    # 提取文件夹名字中带有 '_' 的文件夹
    folders = [folder for folder in folders if '_' in folder]
    folders = [folder for folder in folders if group in folder]
    time_total = 0
    time_iter_average = [0] * 15
    coverage_iter_average = [0] * 15
    cnt_iter = [0] * 15
    cnt_total = 0
    time_average = 0


    # 遍历 folders 中的每一个文件夹
    for folder in folders:
        print("Processing folder: ", folder)
        model_type = folder.split('_')[0] + '_' + folder.split('_')[1]
        obj_folders = os.listdir(os.path.join(path, folder))
        obj_folders = [obj_folder for obj_folder in obj_folders if '_' in obj_folder]
        for obj_folder in obj_folders:
            print("Processing obj folder: ", obj_folder)
            model_name = obj_folder
            full_model_name = model_type + '_' + model_name
            model_path = os.path.join(model_dir, model_type, "pcd", f"{model_name}.pcd")

            # 用于计算点云覆盖率
            ground_truth = o3d.io.read_point_cloud(model_path)
            current_data = o3d.geometry.PointCloud()

            iter_folders = os.listdir(os.path.join(path, folder, obj_folder))
            iter_folders = [iter_folder for iter_folder in iter_folders if '_' in iter_folder]

            # 排序 iter_folders
            iter_folders.sort(key=lambda x: int(x.split('_')[-1]))
            time_current = [0] * len(iter_folders)
            coverage_current = [0] * len(iter_folders)
            for i in range(len(iter_folders)):
                print("Processing iter folder: ", iter_folders[i])
                # 如果 iter_folders[i] 文件夹下面没有文件
                if not os.path.exists(os.path.join(path, folder, obj_folder, iter_folders[i], 'point_cloud.pcd')):
                    print("No point cloud file in this folder")
                    continue

                cnt_total += 1
                cnt_iter[i] += 1
                # 读取文件夹下的 time.txt 文件
                with open(os.path.join(path, folder, obj_folder, iter_folders[i], 'time.txt'), 'r') as f:
                    # 读取文件内容
                    lines = f.readlines()
                    # 遍历每一行
                    for line in lines:
                        # 提取时间
                        time = float(line)
                        time_total += time
                        time_iter_average[i] += time
                        time_current[i] = time

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

            draw_frame(time_current, coverage_current, full_model_name, os.path.join(path, folder, obj_folder))

    time_average = time_total / cnt_total
    for i in range(15):
        time_iter_average[i] /= cnt_iter[i]

    return time_average, time_iter_average

if __name__ == '__main__':
            
    print("Reading data...")
    goups = read_groups(data_dir)
    for group in goups:
        time_average, time_iter_average = process_data(data_dir, group)
        print("Time average: ", time_average)