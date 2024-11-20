import torch
import torch.nn as nn
from torch.nn.functional import dropout
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import transforms
import numpy as np
from tqdm import tqdm
from dataset import VOXELDataset, VOXELDataset2, ToTensor, To3DGrid
from model import MyNBVNetV2
from torch.autograd import Variable
import sys

# device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
device = torch.device('cpu')
learning_rate = 2e-4
batch_size = 64
num_epochs = 500

def eval(datapath):
    test_dataset = VOXELDataset(datapath, transform=transforms.Compose([To3DGrid(), ToTensor()]))
    # test_dataset = VOXELDataset2('../grid_stanford_25.npy', '../label_stanford_25.npy', transform=transforms.Compose([To3DGrid(), ToTensor()]))

    test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=True)
    # print(len(test_dataset))

    model = MyNBVNetV2()
    model = model.to(device)
    # print(model)

    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    all_loss = []

    checkpoint = torch.load('../my_nbv4/full_patch_499.pth.tar')
    model.load_state_dict(checkpoint['state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer'])

    print('EVALUATING')
    model.eval()
    loss = 0
    accuracy = 0
    for sample in test_loader:
        grid = sample[0].to(device)
        label = sample[1].to(device)

        output = model(grid)
        loss += criterion(output, label)
        output[output >= 0.5] = 1
        output[output < 0.5] = 0
        # print(output.shape)
        for i in range(label.shape[0]):
            correct1 = 0
            cnt1 = 0
            for j in range(64):
                if label[i][j] == 1 and output[i][j] == 1:
                    correct1 += 1
                    cnt1 += 1
                elif label[i][j] == 1 and output[i][j] == 0:
                    cnt1 += 1
            # accuracy += 1/np.exp(64-(cnt1-c))
            print(cnt1-correct1)

            # print(torch.nonzero(output[i]).shape,  torch.nonzero(label[i]).shape)
            # correct = (output[i]==label[i]).sum().item()
            # accuracy += 1 / np.exp(64-correct)
            # print(64 - correct)

    accuracy /= len(test_dataset)
    loss /= len(test_loader)
    print(f'accuracy: {accuracy}, loss:{loss}')

if __name__ == '__main__':
    eval('../data/SC_label_data/02773838')
