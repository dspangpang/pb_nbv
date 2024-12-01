#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import os
import time
import threading
import math

model_name = "hb_models_see"
current_dir = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/res_data/video/{model_name}/obj_000016"
img_save_dir = f"/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/res_data/video/video_pcd_img"
save_dir = img_save_dir + "/" + model_name
camera_file = os.path.join(save_dir, "camera.json")

# 创建文件夹
if not os.path.exists(img_save_dir + "/" + model_name):
    os.makedirs(img_save_dir + "/" + model_name)

# 文件读写锁
lock = threading.Lock()

# 设置随机数种子
np.random.seed(5)

def generate_unique_color():
    # 生成一个随机颜色
    color = np.random.rand(3)
    return color
    
def run_visualization():
    global exit_flag
    global save_flag
    global file_cnt
    global camera_flag
    global vis

    # 设置一个循环，使用spin_once方法
    while not exit_flag:
        if save_flag:
            vis.poll_events()
            vis.update_renderer()
            save_flag = False
            save_path = os.path.join(save_dir, model_name + "_" + str(file_cnt) + ".png")
            vis.capture_screen_image(save_path)
            print("save image!")
            
        if camera_flag:
            camera_flag = False
            # 保存当前摄像机的位置方向参数到文件
            # 获取当前摄像机的参数
            camera_params = vis.get_view_control().convert_to_pinhole_camera_parameters()
            # 保存摄像机参数到文件
            o3d.io.write_pinhole_camera_parameters(camera_file, camera_params)
            print("save camera!")
    
def monitor_file():
    global vis
    global exit_flag
    global save_flag
    global input_pcd
    global file_cnt
    global camera_flag
    global camera_params
    global have_camera_config
    
    # 用于计数
    print("monitor_file start !")
    file_cnt = 0
    while not exit_flag:
        # 暂停一段时间，可以根据需要调整检测频率
        time.sleep(0.5)  # 1秒检查一次
        file = current_dir + "/iter_" + str(file_cnt+1) + "/" + "point_cloud.pcd"
        print(f"file: {file}")
        pcd = o3d.io.read_point_cloud(file)
        # 点云滤波
        # pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=1)
        pcd.paint_uniform_color(generate_unique_color())
        input_pcd += pcd
        
        if(file_cnt == 0):
            vis.add_geometry(input_pcd)
        else:
            vis.add_geometry(input_pcd, reset_bounding_box=False)
        if have_camera_config:
            vis.get_view_control().convert_from_pinhole_camera_parameters(camera_params, True)
        
        vis.update_renderer()
        vis.update_renderer()
        vis.update_renderer()

        # if file_cnt == 0 or file_cnt == 1 or file_cnt == 3 or file_cnt == 6 or file_cnt == 9:
        if file_cnt < 10:
            if file_cnt == 0:
                input("Press Enter to continue...")
                # open3d 保存当前窗口的截图
                lock.acquire()
                save_flag = True
                lock.release()
                time.sleep(0.5)
            else:
                lock.acquire()
                save_flag = True
                lock.release()
                time.sleep(0.5)
                
        file_cnt +=1
        
        if file_cnt == 10:
            input("Press Enter to continue...")
            # 读取当前摄像机的位置方向参数到文件
            lock.acquire()
            camera_flag = True
            lock.release()
            exit_flag = True  

def key_callback(vis, action, mods):
    global exit_flag
    """
    按键回调函数。
    当按下 'Q' 或 'q' 键时，关闭可视化窗口。
    """
    # GLFW_PRESS is 1
    if action == 1:
        print("key_callback: action = %d, mods = %d" % (action, mods))
        if mods == 0:
            lock.acquire()
            exit_flag = True
            lock.release()
            vis.close()
        
if __name__ == '__main__':
    
    exit_flag = False
    save_flag = False
    camera_flag = False
    file_cnt = 0
    camera_params = None
    have_camera_config = False
    
    # 创建可视化窗口
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    
    vis.register_key_action_callback(ord('Q'), key_callback)
    vis.register_key_action_callback(ord('q'), key_callback)
    
    input_pcd = o3d.geometry.PointCloud()
    vis.add_geometry(input_pcd)
    
    # 通过读取文件获取摄像机参数
    # 判断文件是否存在
    if os.path.exists(camera_file):
        have_camera_config = True
        camera_params = o3d.io.read_pinhole_camera_parameters(camera_file)
        vis.get_view_control().convert_from_pinhole_camera_parameters(camera_params, True)
    else:
        print("camera file not exist!")
    
    # 在窗口中添加文字
    # 新建一个线程，用于监控文件夹
    t = threading.Thread(target=monitor_file)
    t.start()
    
    run_visualization()

            
