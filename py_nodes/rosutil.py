'''
ArgParser was removed as there are no global parameter server and parameters are private to nodes in ROS2. 
msg_to_torch is used in interestingness_node.py
'''

import torch
import numpy as np

def msg_to_torch(data, shape=np.array([-1])):
    return torch.from_numpy(data).view(shape.tolist())

def torch_to_msg(tensor):
    return [tensor.view(-1).cpu().numpy(), tensor.shape]
