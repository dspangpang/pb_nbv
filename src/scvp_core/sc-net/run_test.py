import sys
import os
import numpy as np
import yaml



model_path = "/root/work_place/pb_nbv/src/gazebo_benchmark_env/env_startup/models/"
model_type = "stanford_models"
name_of_pcd = "obj_000001.pcd"
log_path = "/root/work_place/pb_nbv/src/scvp_core/sc-net/log"
data_path = "/root/work_place/pb_nbv/src/scvp_core/sc-net/data"

print('Model Path:', model_path)
print('Name of PCD:', name_of_pcd)

name_of_model = os.path.join(model_path, model_type, "pcd", name_of_pcd)
print('testing ' + name_of_model)
while not os.path.isfile(os.path.join(data_path, name_of_model + '.txt')):
    pass
os.system('python3 /root/work_place/pb_nbv/src/scvp_core/sc-net/eval_single_file.py' + name_of_model)
with open(log_path + '/ready.txt', 'a') as f:
    f.close()
print('testing ' + name_of_model + ' over.')

