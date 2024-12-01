import torch
import torch.nn as nn
from torch.nn.functional import dropout
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import transforms
import numpy as np
from tqdm import tqdm
from dataset import VOXELDataset, VOXELDataset2, ToTensor, To3DGrid
from model import MyNBVNetV2, MyNBVNetV3
from torch.autograd import Variable
import sys
import os
import time
from scipy.ndimage import zoom

# 从环境变量中获取工作目录
work_dir = os.environ['WORK_DIR']


# device = torch.device('cuda:1' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')

# learning_rate = 2e-4
# batch_size = 64
# num_epochs = 500

log_cache_path = f'{work_dir}src/scvp_core/sc-net/log/'
data_cache_path = f'{work_dir}src/scvp_core/sc-net/data/'
# 创建文件夹
os.makedirs(log_cache_path, exist_ok=True)
os.makedirs(data_cache_path, exist_ok=True)

name_of_model = str(sys.argv[1])


def eval(datapath):

    # 读取数据
    test_data = np.genfromtxt(datapath)
    print(f"Original test_data shape: {test_data.shape}")
    
    # 确保数据大小为 32768
    target_size = 32768
    target_shape = (32, 32, 32)
    if test_data.size > target_size:
        # 假设原始数据是立方体的
        original_size = int(round(test_data.size ** (1/3)))
        test_data = test_data.reshape((original_size, original_size, original_size))
        print(f"Reshaped original test_data to: {test_data.shape}")
        
        # 计算缩放因子
        scale_factors = [t / s for t, s in zip(target_shape, test_data.shape)]
        test_data = zoom(test_data, scale_factors, order=1)
        print(f"Resampled test_data shape: {test_data.shape}")
    
    test_data = test_data.reshape(1, 1, *target_shape)

    # test_data = np.genfromtxt(datapath).reshape(1, 1, 32, 32, 32)
    test_data = torch.from_numpy(test_data).to(torch.float32)

    model = MyNBVNetV3()
    model = model.to(device)

    checkpoint = torch.load(f'{work_dir}src/scvp_core/sc-net/pt/last.pth.tar',map_location = torch.device('cpu'), weights_only=True)
    model.load_state_dict(checkpoint['state_dict'])

    print('EVALUATING')
    model.eval()
    grid = test_data.to(device)
    
    startTime = time.time()
    
    output = model(grid)

    endTime = time.time()
    print('run time is ' + str(endTime-startTime))
    np.savetxt(data_cache_path+name_of_model+'_time.txt',np.asarray([endTime-startTime]))
    
    output[output >= 0.5] = 1
    output[output < 0.5] = 0
    # print(output.shape)

    return output
    

if __name__ == '__main__':

    model_path = os.path.join(data_cache_path, name_of_model + '.txt')

    pred = eval(model_path)
    ans = []
    for i in range(pred.shape[1]):
        if pred[0][i] == 1:
            ans.append(i)


    # 如果 ans 的长度为 小于10 则用最后一个数填充
    while len(ans) < 10:
        ans.append(ans[-1])

    # 如果 ans 中存在0 则删除
    while 0 in ans:
        ans.remove(0)
        
    # 从 name_of_model 提取 / 之后的字符串
    model_name = name_of_model.split('/')[-1]
    # 如果文件夹不存在则创建
    os.makedirs(log_cache_path + name_of_model, exist_ok=True)
    np.savetxt(log_cache_path + name_of_model + "/" + model_name +'.txt',ans,fmt='%d')
