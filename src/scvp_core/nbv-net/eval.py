from torch.nn.functional import dropout
from nbvnet import NBV_Net
import torch
import numpy as np


def get_single_view_point(path):
    net = NBV_Net(dropout_prob=0)
    checkpoint = torch.load('my_checkpoint.pth.tar')
    net.load_state_dict(checkpoint['net'])
    grid = np.genfromtxt(path).reshape(1, 1, 32, 32, 32)
    grid = torch.tensor(grid, dtype=torch.float32)
    ids = net(grid)
    return ids
    

if __name__ == '__main__':
    ids = get_single_view_point('../data/NBV-Net_lable_data/LM2/grid_2.txt')
    print(ids.argmax(dim=1))
    # print(net)

    
