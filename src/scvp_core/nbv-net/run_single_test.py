import sys
import os
from torch.nn.functional import dropout
from nbvnet import NBV_Net
import torch
import numpy as np
import time
from scipy.ndimage import zoom

# 从环境变量中获取工作目录
work_dir = os.environ['WORK_DIR']

pth_path = f'{work_dir}src/scvp_core/nbv-net/my_checkpoint.pth.tar'
data_cache_path = f'{work_dir}src/scvp_core/nbv-net/data/'
log_cache_path = f'{work_dir}src/scvp_core/nbv-net/log/'

model = str(sys.argv[1])

max_iteration = int(sys.argv[2])

# 清空 data_cache_path+model下的所有文件
print('cleaning '+data_cache_path)
os.system('rm -rf '+data_cache_path+"*")
print('cleaning '+log_cache_path)
os.system('rm -rf '+log_cache_path+"*")

def get_single_view_point(path):
    net = NBV_Net(dropout_prob=0)
    checkpoint = torch.load(pth_path, map_location=torch.device('cpu'), weights_only=True)
    net.load_state_dict(checkpoint['net'])
    
    # 读取数据
    grid = np.genfromtxt(path)
    print(f"Original grid shape: {grid.shape}")
    
    # 确保数据大小为 32768
    target_size = 32768
    target_shape = (32, 32, 32)
    if grid.size > target_size:
        # 假设原始数据是立方体的
        original_size = int(round(grid.size ** (1/3)))
        grid = grid.reshape((original_size, original_size, original_size))
        print(f"Reshaped original grid to: {grid.shape}")
        
        # 计算缩放因子
        scale_factors = [t / s for t, s in zip(target_shape, grid.shape)]
        grid = zoom(grid, scale_factors, order=1)
        print(f"Resampled grid shape: {grid.shape}")
    
    grid = grid.reshape(1, 1, *target_shape)
    grid = torch.tensor(grid, dtype=torch.float32)
    ids = net(grid)
    return ids

print('testing '+ model)
iteration = 0
while iteration<max_iteration:
    print(data_cache_path+model+'_'+str(iteration)+'.txt')
    while os.path.isfile(data_cache_path+model+'_'+str(iteration)+'.txt')==False:
        pass
    time.sleep(1)
    ids = get_single_view_point(data_cache_path+model+'_'+str(iteration)+'.txt')
    ids = ids.argmax(dim=1)
    # 创建文件夹
    if not os.path.exists(log_cache_path+model):
        os.makedirs(log_cache_path+model)
    np.savetxt(log_cache_path+model+'_'+str(iteration)+'.txt',ids,fmt='%d')
    f = open(log_cache_path+'ready.txt','a')
    f.close()
    iteration += 1
print('testing '+ model + ' over.')
