import torch
from model import SCVP
import sys
import time
import numpy as np
import os
from scipy.ndimage import zoom

DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')


def eval_pth(voxel, checkpoint_path, view_state=None):
    test_data = voxel.to(torch.float32)

    model = SCVP(net_type='SCVP' if view_state==None else "MASCVP").to(DEVICE)

    checkpoint = torch.load(checkpoint_path,map_location = torch.device('cpu'))
    model.load_state_dict(checkpoint['state_dict'])

    print('EVALUATING')
    model.eval()
    grid = test_data.to(DEVICE)
    if view_state is not None:
        print('MA-SCVP')
        view_state = view_state.to(torch.float32)
        view_state = view_state.to(DEVICE)

    startTime = time.time()
    
    output = model(grid, view_state)

    endTime = time.time()
    print('run time is ' + str(endTime-startTime))
    np.savetxt('./run_time/'+name_of_model+'.txt',np.asarray([endTime-startTime]))
    
    output[output >= 0.5] = 1
    output[output < 0.5] = 0
    # print(output.shape)

    return output
    

if __name__ == '__main__':
    pth_path = str(sys.argv[1])
    name_of_model = str(sys.argv[2])

    voxel_path = './data/'+name_of_model+'_voxel.txt'
    vss_path = './data/'+name_of_model+'_vs.txt'

    x = np.genfromtxt(voxel_path)
    target_shape = (32, 32, 32)
    target_size = np.prod(target_shape)
    if x.size == target_size:
        x = x.reshape(1, 1, *target_shape)
    else:
        original_size = int(round(x.size ** (1 / 3)))
        x = x.reshape((original_size, original_size, original_size))
        scale = [t / s for t, s in zip(target_shape, x.shape)]
        x = zoom(x, scale, order=1).reshape(1, 1, *target_shape)
    x = torch.from_numpy(x)

    vs = None
    if os.path.getsize(vss_path) != 0:
        vs = np.genfromtxt(vss_path).reshape(1, 1, 32)
        vs = torch.from_numpy(vs)
    
    pred = eval_pth(x, pth_path, vs)
    
    ans = []
    for i in range(pred.shape[1]):
        if pred[0][i] == 1:
            print(i)
            ans.append(i)

    # 若没有任何激活结果，默认补一个 0，避免后续访问 ans[-1] 报错
    if not ans:
        ans.append(0)

    # 不足 10 帧时重复最后一帧补齐
    while len(ans) < 10:
        ans.append(ans[-1])

    # 剔除编号 0 的视角，但至少保留 1 个元素
    while 0 in ans and len(ans) > 1:
        ans.remove(0)

    np.savetxt('./log/'+name_of_model+'.txt',ans,fmt='%d')
