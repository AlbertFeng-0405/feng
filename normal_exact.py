import os
from types import coroutine
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import pickle

obj_file_path = '/home/albert/obj_model/house_obj/house/textures/Farm_house.obj'
norm_path = '/home/albert/obj_model/house_obj/house/textures/output.txt'

def cal_normal(pt_0,pt_1,pt_2):
    vec_0 = pt_1 - pt_0
    vec_1 = pt_2 - pt_1
    normal = np.cross(vec_0,vec_1)
    norm = np.linalg.norm(normal)
    print(norm)

    return normal

def get_key (dict, value):
    
    return [key for key, ve in dict.items() if ve == value ]

def find_face_pt(file_path, vn_path):

    faces_co = {}
    face_center = {}

    thefile=open(file_path)
    lines_num = len(thefile.readlines())

    with open(file_path) as file:
        vertices = []
        vn_list = []
        faces = []

        for i in range(lines_num):
            line = file.readline()
            line = line.replace('\n', '')
            strs = line.split(" ")
            if strs[0] == 'v':
                vertices.append(np.array([float(strs[2]),float(strs[3]),float(strs[4])]))
            if strs[0] == 'f':
                a = [int(i) for i in strs[1].split('/')]
                b = [int(i) for i in strs[2].split('/')]
                c = [int(i) for i in strs[3].split('/')]
                face_temp = [a[::2],b[::2],c[::2]]#[[a_v_index,a_normal_index],[b_v_index,b_normal_index],[c_v_index,c_normal_index]]
                faces.append(np.array(face_temp))
    with open(vn_path,'r') as f:
        for line in f.readlines():
            line = line.strip('\n')  #去掉列表中每一个元素的换行符
            line = line.split(' ')
            temp_pt = np.array([float(line[1]),float(line[2]),float(line[3])])
            temp_norm = np.linalg.norm(temp_pt)
            if temp_norm == 0:
                temp_norm = 1
            temp_pt = temp_pt/temp_norm
            vn_list.append(temp_pt) 
    
    return vertices, vn_list

if __name__ == '__main__':
    pts, vns = find_face_pt(obj_file_path, norm_path)
    cluster_center = []
    cluster_ref_normal = []
    with open('/home/albert/learn_cpp/kmeans/build/center.txt','r') as f:
        for line in f.readlines():
            line = line.strip('\n') 
            line = line.split(' ')
            temp_pt = np.array([float(line[1]),float(line[2]),float(line[3])])
            cluster_center.append(temp_pt)
    for i in range(len(cluster_center)):
        temp_dis = {}
        temp_list = []
        for j in range(len(pts)):
            temp_dis[j] = np.linalg.norm(cluster_center[i]-pts[j])
            temp_list.append(np.linalg.norm(cluster_center[i]-pts[j]))
        temp_list.sort()
        index = get_key(temp_dis,temp_list[0])[0]
        flag=0
        while (np.mean(vns[index]) == 0):
            index = get_key(temp_dis,temp_list[flag+1])[0]
            flag+1
        cluster_ref_normal.append(vns[index])
    
    print(cluster_ref_normal)

    for i in range(len(cluster_ref_normal)):
        f = open('/home/albert/learn_cpp/kmeans/build/ref.txt','a',buffering = 200)
        strs = cluster_ref_normal[i]
        norm_vec = str(i+1)+' '+str(strs[0])+' '+str(strs[1])+' '+str(strs[2])
        f.write(norm_vec)
        f.write('\n')
        #f.close()

        
