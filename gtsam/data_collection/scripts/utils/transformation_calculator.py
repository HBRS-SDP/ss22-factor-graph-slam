from typing import Sequence
import numpy as np
import numpy.linalg as linalg
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from IPython.core.pylabtools import figsize, getfigs

file_name = '../Extracted_data_from_bag_files/sdp_look_two_3_marker.csv'

def get_RX(theta: float) -> np.ndarray:
    
    cos = np.cos(theta)
    sin = np.sin(theta)
    Rx = np.array(((1,0,0),
                   (0,cos,-sin),
                   (0,sin,cos)))

    return Rx

def get_RY(theta: float) -> np.ndarray:
    
    cos = np.cos(theta)
    sin = np.sin(theta)
    Ry = np.array(((cos , 0  ,sin),
                   ( 0  , 1  ,0),
                   (-sin, 0  ,cos)))

    return Ry

def get_RZ(theta: float) -> np.ndarray:
    
    cos = np.cos(theta)
    sin = np.sin(theta)
    Rz = np.array(((cos , -sin  ,0),
                   ( sin, cos  ,0),
                   (0, 0  ,1)))

    return Rz
    
def get_homogeneous_transform(input):

    rotation_angles =np.array(input[3:6])
    translation=np.array(input[0:3])
    W=input[6]

    rot = rotation_angles

    Rx = get_RX(rot[2]) # rotation about X axis
    Ry = get_RY(rot[1]) # rotation about Y axis
    Rz = get_RZ(rot[0]) # rotation about Z axis
 
    h_stack = np.array([[0],[0],[0]])
    v_stack = np.array([0,0,0,W])
    Rotational_matrix = np.dot(Rz, np.dot(Ry, Rx)) 
    
    r = Rotational_matrix
    r = np.hstack((r,h_stack))
    r = np.vstack((r,v_stack))
    
    trans = np.array(([translation[0]],
                      [translation[1]],
                      [translation[2]]))
    
    Translational_matrix = np.array(((1,0,0),(0,1,0),(0,0,1)))

    t = Translational_matrix
    t = np.hstack((t,trans))
    t = np.vstack((t,v_stack))
    
    H_matrix = t.dot(r)
    
    return H_matrix

def transforms():
    input = [0.239, 0.0, 0.071, 0.0, 0.0,0.0,1]
    b_T_apl = get_homogeneous_transform(input)

    input = [0,0,0,0,0,-0.38268343236488267, 0.9238795325113726]
    apl_T_arl = get_homogeneous_transform(input)

    input = [-0.022,0.0,0.037,0,0,0,1]
    arl_T_a0 = get_homogeneous_transform(input)

    input = [0.024,0.0,0.096,0.0,0.0,0.37421911645649647,0.9273403112549993]
    a0_T_a1 = get_homogeneous_transform(input)

    input = [0.033,0.0,0.019,0,0.528847175314683, 0.0, 0.8487170701486337]
    a1_T_a2 = get_homogeneous_transform(input)

    input = [0,0,0.155,0,-0.18396078610708805,0,0.9829335832979063]
    a2_T_a3 = get_homogeneous_transform(input)

    input = [0.0,0.0,0.135,0.0,0.722613797908215, 0, 0.6912519794348986]
    a3_T_a4 = get_homogeneous_transform(input)

    input = [-0.002, 0, 0.13 , 0,0, 0.7120631548522023,  0.7021154203561754]
    a4_T_a5 = get_homogeneous_transform(input)
    
    input = [-0.005, -0.008,0.009,  -0.49999999999755174, -0.5,-0.5,  0.5000000000024483]
    a5_T_cl = get_homogeneous_transform(input)
    
    input = [-0.000208392972126603, 0.014731519855558872,  0.00020157775725238025, -0.014435658231377602, -0.00020775734446942806, -0.0008906595758162439,  0.9998953938484192]
    cl_T_ccf = get_homogeneous_transform(input)

    b_T_ccf = cl_T_ccf.dot(a5_T_cl).dot(a4_T_a5).dot(a3_T_a4).dot(a2_T_a3).dot(a1_T_a2).dot(a0_T_a1).dot(arl_T_a0).dot(apl_T_arl).dot(b_T_apl)

    return b_T_ccf

def data_loader():
    
    global file_name

    my_data = np.genfromtxt(file_name, delimiter=',',skip_header=1)
    
    my_data = np.delete(my_data,[1,2,10,11,12,13,14,15],1)

    return my_data

def yaw_angle_calculator(data):

    b_T_ccf = transforms()
    data = data_loader()
    ccf_T_F_seq = get_homogeneous_transform(data)
    b_T_F_seq = ccf_T_F_seq.dot(b_T_ccf)
    rotation = R.from_matrix(b_T_F_seq[0:3,0:3]) # rotation matrix of frame F relative to frame B
    yaw_angle = rotation.as_euler('xyz', degrees=True)[2]

    return yaw_angle

def dict_generator():

    our_data= data_loader()
    ids_in_data = our_data[:,0]
    LM_dict ={}
    cnt=1
    for i in ids_in_data:
        if(i not in list(dict.keys(LM_dict))):
            LM_dict[i]="L"+str(cnt)
            cnt+=1
    print(LM_dict)

    return LM_dict

dict_generator()