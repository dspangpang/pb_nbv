import sys
import numpy as np
import yaml
import os
from torch.nn.functional import dropout
import torch
import time
from scipy.ndimage import zoom

data_cache_path = '/root/work_place/pb_nbv/src/scvp_core/sc-net/data/'
log_cache_path = '/root/work_place/pb_nbv/src/scvp_core/sc-net/log/'

model = str(sys.argv[1])

max_iteration = int(sys.argv[2])

# 清空 data_cache_path+model下的所有文件
print('cleaning '+data_cache_path)
os.system('rm -rf '+data_cache_path+"*")
print('cleaning '+log_cache_path)
os.system('rm -rf '+log_cache_path+"*")

print('testing ' + model)
while not os.path.isfile(os.path.join(data_cache_path, model + '.txt')):
    pass
os.system('python3 /root/work_place/pb_nbv/src/scvp_core/sc-net/eval_single_file.py ' + model)
with open(log_cache_path + '/ready.txt', 'a') as f:
    f.close()
print('testing ' + model + ' over.')

