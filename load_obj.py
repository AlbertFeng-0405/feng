import os
from types import coroutine
import numpy as np
from numpy.core.defchararray import _center_dispatcher, center
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import pickle

obj_file_path = '/home/albert/obj_model/house_obj/house/textures/Farm_house.obj'
file_path = '/home/albert/optm_traj_reconstruction_demo/cluster'
norm_path = '/home/albert/optm_traj_reconstruction_demo/normal'


def Kmeans(coord_list, K):
    clf = KMeans(n_clusters=K, max_iter=300, n_init=40, \
                    init='k-means++',n_jobs=-1)
    pred = clf.fit_predict(coord_list)  # clustering
 
    centers = clf.cluster_centers_ # centers
    labels = clf.labels_   # label of each points in coord_list

    x = coord[:, 0]
    y = coord[:, 1]
    z = coord[:, 2]

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(x, y, z,c=pred)
    plt.show()

    return centers, labels

def get_key (dict, value):
    temp = {}
    result = []
    for k,v in dict.items():
        temp[k] = abs(np.sum(v-value))
        result.append(abs(np.sum(v-value)))
    mins = min(result)
    print(mins)
    
    return [key for key, ve in temp.items() if ve == mins ]
    


if __name__ == '__main__':

    faces_center = {}
    faces_co = {}
    label_faces = {}
    norm ={}
    cen = {}
    center_normal = []

    thefile=open(obj_file_path)
    lines_num = len(thefile.readlines())

    with open(obj_file_path) as file:
        vertices = [[0,0,0]]
        vn_list = [[0,0,0]]
        faces = []

        for i in range(lines_num):
            line = file.readline()
            line = line.replace('\n', '')
            strs = line.split(" ")
            if strs[0] == 'v':
                vertices.append(np.array([float(strs[2]),float(strs[3]),float(strs[4])]))
            if strs[0] == 'vn':
                vn_list.append(np.array([float(strs[1]),float(strs[2]),float(strs[3])]))
            if strs[0] == 'f':
                a = [int(i) for i in strs[1].split('/')]
                b = [int(i) for i in strs[2].split('/')]
                c = [int(i) for i in strs[3].split('/')]
                face_temp = [a[::2],b[::2],c[::2]]#[[a_v_index,a_normal_index],[b_v_index,b_normal_index],[c_v_index,c_normal_index]]
                faces.append(np.array(face_temp))
    
    for i in range(len(faces)):
        a_0 = vertices[faces[i][0][0]]# a coordinates
        a_1 = vn_list[faces[i][0][1]]# a normal
        b_0 = vertices[faces[i][1][0]]
        b_1 = vn_list[faces[i][1][1]]
        c_0 = vertices[faces[i][2][0]]
        c_1 = vn_list[faces[i][2][1]]

        coord_center = (a_0+b_0+c_0)/3
        normal_center = (a_1+b_1+c_1)/3

        faces_center[i]=np.array([coord_center,normal_center])
        norm[i] = normal_center
        cen[i] = coord_center

        faces_co[i]=[list(a_0),list(b_0),list(c_0)]
    
    coord = []
    for j in faces_center.values():
        coord.append(j[0])
    coord = np.array(coord)

    #center_list,label_list = Kmeans(coord,50)

    # with open('/home/albert/optm_traj_reconstruction_demo/label.pkl','rb') as f:
    #     #pickle.dump(label_list,f)
    #     label_list = pickle.load(f)
    
    # with open('/home/albert/optm_traj_reconstruction_demo/center.pkl','rb') as f:
    #     #pickle.dump(center_list,f)
    #     center_list = pickle.load(f)
    
    # # labels = set(label_list)

    # # for i in labels:
    # #     label_faces[i] = []
    
    # # for i in range(len(label_list)):
    # #     for j in labels:
    # #         if label_list[i] == j:
    # #             label_faces[j].append(faces_co[i])
    
    # for i in center_list:
    #     temp_idx = get_key(cen,i)[0]
    #     temp_list = [i[0],i[1],i[2],norm[temp_idx][0],norm[temp_idx][1],norm[temp_idx][2]]
    #     center_normal.append(temp_list)
    
    # # with open('/home/albert/optm_traj_reconstruction_demo/label_faces.pkl','wb') as f:
    # #     pickle.dump(label_faces,f)
    
    # with open('/home/albert/optm_traj_reconstruction_demo/normal.pkl','wb') as f:
    #     pickle.dump(center_normal,f)
    
    # print(len(center_normal))
    # print(center_normal)
    
    # with open('/home/albert/optm_traj_reconstruction_demo/label_faces.pkl','rb') as f:
    #     label_f = pickle.load(f)
    
    # flag = 0
    # for i in range(len(label_f)):
    #     for j in label_f[i]:
    #         f = open(os.path.join(file_path+'/'+str(i)+'.txt'),'a',buffering = 200)
    #         coordinats = str(j[0][0])+' '+str(j[0][1])+' '+str(j[0][2])+' '+str(j[1][0])+' '+str(j[1][1])+' '+str(j[1][2])+' '+str(j[2][0])+' '+str(j[2][1])+' '+str(j[2][2])
    #         flag += 1
    #         f.write(coordinats)
    #         f.write('\n')
    #         #f.close()
    # print(flag)

    with open('/home/albert/optm_traj_reconstruction_demo/normal.pkl','rb') as f:
        normal = pickle.load(f)
    
    print(len(normal))
    for i in normal:
        f = open(os.path.join(norm_path+'/norm.txt'),'a',buffering = 200)
        norm_vec = str(i[0])+' '+str(i[1])+' '+str(i[2])+' '+str(i[3])+' '+str(i[4])+' '+str(i[5])
        f.write(norm_vec)
        f.write('\n')
        # f.close()