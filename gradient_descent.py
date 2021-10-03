import re
from matplotlib import colors
import numpy as np
import matplotlib.pyplot as plt
import cv2
from matplotlib.path import Path
import matplotlib.patches as patches
from rdp import rdp
from bezier_sim import *
import heapq

# find_shape --> vis_smooth --> find iter pt --> get gradient vector --> get delta yaw --> turn delta_yaw
# --> move along x axis in new coord --> judge overlap

def find_shape(list,coe):
    '''
    :coe: control shape pin 
    '''
    x_list=[]
    y_list=[]

    num = len(list)
    cp = num//4
    a = rdp(list[:cp],epsilon=coe)
    b = rdp(list[cp:2*cp],epsilon=coe)
    c = rdp(list[2*cp:3*cp],epsilon=coe)
    d = rdp(list[3*cp:],epsilon=coe)

    shape = a+b+c+d

    for i in shape:
        x_list.append(i[0])
        y_list.append(i[1])
    
    return x_list, y_list

def vis_smooth_shape(x_list,y_list,inserts,k):
    x_curve, y_curve = smoothing_base_bezier(x_list, y_list, k=k, inserted=inserts,closed=True)
    pts = []
    assert len(x_curve) == len(y_curve)
    for i in len(x_curve):
        temp=[x_curve[i],y_curve[i]]
        pts.append(temp)
    
    return pts, x_curve, y_curve

def cal_pt2line(curr_pt,yaw,pt):
    if np.cos(yaw) == 0:
        dis = np.linalg.norm(np.array(pt[0])-np.array(curr_pt[0]))
    else:
        k = np.tan(np.pi/2-yaw)
        dis = np.abs(k*pt[0]-pt[1]-k*curr_pt[0]+curr_pt[1])/(np.sqrt(k**2+1**2))
    
    return dis

def interval_gradient(pts):
    gradient_vec = {}
    for i in range(len(pts)-1):
        gradient_vec[i] = np.array(pts[i+1])-np.array(pts[i])#gradient vector in absolute coord
    
    return gradient_vec
    


def find_iteration_pt(curr_pt,yaw,pts,K):
    dis_list = []
    K_dis = {}
    for i in pts:
        dis_list.append(cal_pt2line(curr_pt=curr_pt,yaw=yaw,pt=i))
    
    k_neighbor = map(dis_list.index,heapq.nsmallest(K,dis_list))
    k_neighbor_list = list(k_neighbor)

    for i in k_neighbor_list:
        temp_dis = np.linalg.norm(np.array(pts[i])-np.array(curr_pt))
        K_dis[i] = temp_dis
    
    iteration_pt_idx = min(K_dis, key=K_dis.get)
    
    return iteration_pt_idx

def get_delta_yaw(iter_idx,grad_vec,yaw):
    x_grad = grad_vec[iter_idx][0]
    y_grad = grad_vec[iter_idx][1]
    grad_yaw_coord = np.array([x_grad*np.cos(yaw)-y_grad*np.sin(yaw),x_grad*np.sin(yaw)+y_grad*np.cos(yaw)])
    unit_base = np.array([1,0])
    product = np.dot(np.array(grad_yaw_coord),np.array(unit_base))

    sign = np.sign(np.cross(np.array(grad_yaw_coord),np.array(unit_base)))

    delta_yaw = sign*np.arccos(product/(np.linalg.norm(grad_yaw_coord)*np.linalg.norm(unit_base)))# yaw_now = yaw+yaw_before

    yaw_new = yaw + delta_yaw

    return yaw_new, delta_yaw #next_time yaw





x = np.array([2, 4, 4, 3, 2])
y = np.array([2, 2, 4, 3, 4])

c = [[1,2],[1.5,2.5],[2,3],[3,9],[4,4]]
e = [[6,5],[2,4]]
c = c+e
d = rdp(c,epsilon=0.1)
print(d)