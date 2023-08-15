#!/usr/bin/env python3
from functools import reduce
import operator
import math
import time
import pathlib
import os
import math
import importlib.util
from matplotlib import pyplot as plt
import numpy as np
import scipy as sp
import scipy.spatial
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import tf
import sys
import yaml
import rospy
import std_msgs
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
# from crazyflie import Crazyflie
import message_filters as mef
from pycrazyswarm import Crazyflie
from crazyswarm.msg import Position,position_share,GenericLogData





def str_to_class(str):
    return getattr(sys.modules[__name__], str)

len_val=2.0

def converting_short(q):
    q[0]=q[0]/len_val
    q[1]=q[1]/len_val
    q=q/2
    q=q+np.array([0.5,0.5])
    return np.array(q,dtype=np.float32)

def converting_long(q):
    q=q-np.array([0.5,0.5])
    q=q*2
    q[0]=q[0]*len_val
    q[1]=q[1]*len_val
    return np.array(q,dtype=np.float32)

class agent(Crazyflie):
    gain_gamma = 1.9
    gain_zeta = 1.2
    gain_delta = 1
    a_min = 0.1
    a_max = 100
    stepsize = 0.05
    alpha=0.001
    phi_max=500
    control_gain = 2.3
    nx = 7
    ny = 7
    mm=9

    def __init__(self,id,initialPosition,tf,prefixs=None,cfID=None,centroids=None,target=None,
                 position_set=None,pub=None,voronoi_vertices=None, time=None, 
                 unmod_goal=None, position_local=None, neigb_agent=None, a_hat=None, cap_lamda=None, 
                 small_lamda=None, temp2=None, centroid=None, control_input=None, comm_agent=None,xx=None,yy=None,
                 u_bar=None,basis=None,f_i=None,coverage_cost=None,true_centroid=None,sen_phi=None):
        super().__init__(id,initialPosition,tf)
        self.neigb_agent = neigb_agent
        self.cfID=cfID
        self.prefixs=prefixs
        self.pub= rospy.Publisher(self.prefix+'/position_ahat_pub',position_share,queue_size=1)
        self.a_hat = a_hat
        self.comm_agent = comm_agent
        self.cap_lamda = cap_lamda
        self.small_lamda = small_lamda
        self.position_local = position_local
        self.temp2 = temp2
        self.centroid = centroid
        self.control_input = control_input
        self.time = time
        self.unmod_goal = unmod_goal
        self.sen_phi = sen_phi
        self.voronoi_vertices = voronoi_vertices
        self.xx=xx
        self.yy=yy
        self.position_set = Position()
        self.target=target
        self.centroids=centroids
        self.u_bar=u_bar
        self.f_i=f_i
        self.basis=basis
        self.coverage_cost=coverage_cost
        self.true_centroid=true_centroid
        

    def fun_a_hat_dot(self, t, ahat):
        delly=agent.gain_delta
        n_nie = len(self.neigb_agent)
        temp12 = agent.gain_gamma*(np.matmul(self.cap_lamda, ahat)-self.small_lamda)
        bas=np.matrix(self.basis).T
        temp =-temp12-((agent.gain_gamma*bas*(np.matmul(bas.T,ahat)-self.sen_phi)))
        temp =temp-(agent.gain_zeta*((n_nie*ahat)-self.temp2))
        temp=temp-agent.control_gain*(self.f_i)
        # print('printing the temp in fun_ahat_dot {}'.format(temp))
        temp=delly*temp
        return temp

    def para_updatey(self, pos_dict, ahat_dict):
        n_nie = len(self.neigb_agent)
        a_hat = np.matrix(self.a_hat).T
        temp1 = np.zeros((len(a_hat),1))
        temp2 = np.zeros((len(temp1),1))
        for i in range(n_nie):
            cfID = self.neigb_agent[i]
            Lij = np.linalg.norm(self.position_local - pos_dict[cfID])
            temp1 =temp1+ Lij*(a_hat-np.matrix(ahat_dict[cfID]).T)
            temp2 = temp2+ np.matrix(ahat_dict[cfID]).T
        self.temp2 = temp2

        temp=-(agent.gain_gamma *(np.matmul(self.cap_lamda, a_hat)-self.small_lamda))
        temp =temp-(agent.gain_zeta*temp1)
        i_proj = np.zeros(len(a_hat))
        
        for i in range(len(temp1)):
            if a_hat[i,0] > agent.a_min and a_hat[i,0] < agent.a_max:
                i_proj[i] = 0
            elif a_hat[i,0] == agent.a_min and temp[i,0] >= 0:
                i_proj[i] = 0
            elif a_hat[i,0] == agent.a_max and temp[i,0] <= 0:
                i_proj[i] = 0
            else:
                i_proj[i] = 1

        out2 = self.rk4_method(self.fun_a_hat_dot, a_hat, self.time)
        a_hat=np.squeeze(np.asarray(a_hat))
        out2=np.squeeze(np.asarray(out2))
        for i in range(len(a_hat)):
            if i_proj[i] == 1:
                out2[i] = a_hat[i]
        out2[out2 >= agent.a_max] = agent.a_max
        out2[out2 <= agent.a_min] = agent.a_min
        self.a_hat = out2
        # print('^^^^^^^^^^^^^^^^^^^^^^^^^^^')
        # print('printing a_hat {}'.format(self.a_hat))

    def rk4_method(self, funcs, y, t):
        h = agent.stepsize
        k_1 = funcs(t, y)
        k_2 = funcs((t+0.5*h), (y+0.5*h*k_1))
        k_3 = funcs((t+0.5*h), (y+0.5*h*k_2))
        k_4 = funcs((t+h), (y+k_3*h))
        out1 = y+((1/6)*(k_1+2*k_2+2*k_3+k_4)*h)
        return out1

    
    # def position_generation(self,past_pos,pres_pos,t):


    def f_cap_lamda(self, t, cap_lam):
        cur_pos = np.squeeze(np.asarray(self.position_local))
        wei = np.linalg.norm(self.control_gain*(self.centroid-cur_pos))**2
        by = np.matrix(self.basis).T
        alpha=self.alpha
        temp = (-alpha*(cap_lam))+(wei*np.matmul(by, by.T))
        return temp

    def f_small_lamda(self, t, small_lam):
        cur_pos = np.squeeze(np.asarray(self.position_local))
        wei = np.linalg.norm(self.control_gain*(self.centroid-cur_pos))**2
        alpha=self.alpha
        by = np.matrix(self.basis).T
        temp = (-alpha*(small_lam))+(wei*by*self.sen_phi)
        return temp   

    def state_update(self):
        current_pos = np.squeeze(np.asarray(self.position_local))
        self.sen_phi, self.basis = self.sensory_f(current_pos)
        self.control_input = self.control_gain*(self.centroid-current_pos+self.u_bar)
        self.find_f(current_pos)
        self.cap_lamda = self.rk4_method(self.f_cap_lamda, self.cap_lamda, self.time)
        self.small_lamda = self.rk4_method(self.f_small_lamda, self.small_lamda, self.time)

    def find_f(self,posi):
        nx = agent.nx
        ny = agent.ny
        mm=agent.mm
        current_pos = np.squeeze(np.asarray(posi))
        current_pos=np.matrix(current_pos).T
        verty = self.vertices
        del_x = (max(verty[:, 0])-min(verty[:, 0]))/nx
        del_y = (max(verty[:, 1])-min(verty[:, 1]))/ny
        del_area = del_x*del_y
        x_pt = np.arange(min(verty[:, 0]), max(verty[:, 0])+del_x, del_x)
        y_pt = np.arange(min(verty[:, 1]), max(verty[:, 1])+del_y, del_y)

        f_i=np.zeros((agent.mm,1))
        poly1 = Polygon(verty.tolist())
        boomi=np.matrix([[0.0,0.0],[1.,0.],[1.,1.0],[0.0,1.],[0.0,0.0]])
        poly3=Polygon(boomi)
        for i in range(len(x_pt)-1):
            for j in range(len(y_pt)-1):
                temp_xt = np.array([x_pt[i], x_pt[i+1]])
                temp1 = np.vstack((np.vstack(
                    (temp_xt, np.ones(2)*y_pt[j])).T, np.vstack((temp_xt, np.ones(2)*y_pt[j+1])).T))
                temp_temp = agent.sorty_square(temp1)
                temp=np.vstack((temp_temp,temp_temp[0,:]))
                poly2 = Polygon(temp.tolist())
                if poly1.contains(poly2) and poly3.contains(poly2):
                    center = np.mean(temp_temp, axis=0)
                    center=np.matrix(center).T
                    centroid=np.matrix(np.squeeze(np.asarray(self.centroid))).T
                    tempyu=centroid-(current_pos)
                    phi, basis_func = agent.sensory_f(np.squeeze(np.asarray(center)))
                    bass=np.matrix(basis_func).T
                    f_i+=bass*np.matmul((center-current_pos).T,tempyu)    
        self.f_i=f_i


    def find_cent(self):
        nx = agent.nx
        ny = agent.ny
        a_min=agent.a_min
        a_max=agent.a_max
        mm=agent.mm
        current_pos = np.squeeze(np.asarray(self.position_local))
        current_pos=np.matrix(current_pos).T
        a_hat = np.matrix(self.a_hat).T
        verty = self.vertices
        del_x = (max(verty[:, 0])-min(verty[:, 0]))/nx
        del_y = (max(verty[:, 1])-min(verty[:, 1]))/ny
        del_area = del_x*del_y
        x_pt = np.arange(min(verty[:, 0]), max(verty[:, 0])+del_x, del_x)
        y_pt = np.arange(min(verty[:, 1]), max(verty[:, 1])+del_y, del_y)

        store_num = np.zeros((2,1))
        store_den = 0
        true_store_num=np.zeros((2,1))
        true_store_den=0
        coverage_cost=0
        store_k_bar=0
        store_u_bar=np.zeros((2,1))
        poly1 = Polygon(verty.tolist())
        boomi=np.matrix([[0.0,0.0],[1.,0.],[1.,1.0],[0.0,1.],[0.0,0.0]])
        poly3=Polygon(boomi)
        for i in range(len(x_pt)-1):
            for j in range(len(y_pt)-1):
                temp_xt = np.array([x_pt[i], x_pt[i+1]])
                temp1 = np.vstack((np.vstack(
                    (temp_xt, np.ones(2)*y_pt[j])).T, np.vstack((temp_xt, np.ones(2)*y_pt[j+1])).T))
                temp_temp = agent.sorty_square(temp1)
                temp=np.vstack((temp_temp,temp_temp[0,:]))
                poly2 = Polygon(temp.tolist())
                if poly1.contains(poly2) and poly3.contains(poly2):
                    center = np.mean(temp_temp, axis=0)
                    center=np.matrix(center).T
                    phi, basis_func = agent.sensory_f(np.squeeze(np.asarray(center)))
                    basis_func=np.matrix(basis_func).T
                    temphi = np.matmul(basis_func.T, a_hat)*del_area
                    store_num =store_num + (center*temphi)
                    # print(store_num)
                    store_den =store_den+ temphi
                    aaa = np.hstack([a_min, a_min,a_max,a_min,a_min,a_min,a_max, a_min,a_min])
                    true_temphi=np.matmul(basis_func.T, aaa)*del_area
                    true_store_num =true_store_num + (center*true_temphi)
                    # print(store_num)
                    true_store_den =true_store_den+ true_temphi
                    coverage_cost+=(np.matmul(basis_func.T,aaa)*del_area*del_area*(np.linalg.norm(center-current_pos)**2))
                    store_k_bar+=(np.linalg.norm(current_pos-center)**2)*(del_area*del_area)*agent.phi_max
                    store_u_bar+=(center-current_pos)*(del_area)
        store_u_bar=store_u_bar*store_k_bar  
        try:   
            self.centroid = np.squeeze(np.asarray(store_num/store_den)) 
            self.coverage_cost=coverage_cost
            self.true_centroid=np.squeeze(np.asarray(true_store_num/true_store_den)) 
            self.u_bar=np.squeeze(np.asarray(store_u_bar))

        except ZeroDivisionError as err:
            self.coverage_cost=coverage_cost
            pass

    @staticmethod
    def sensory_f(q):
        m = 9
        var = 0.18**2
        a_min = 0.1
        a_max = 100
        cond = np.array([0, 1])
        lim = 1/6
        mu_x = np.array([lim, lim*3, lim*5])
        mu = np.hstack([np.array([mu_x, lim*5*np.ones(3).T]), np.array([mu_x,
                       lim*3*np.ones(3).T]), np.array([mu_x, lim*np.ones(3).T])])
        basis_func = np.zeros(9)
        for i in range(m):
            basis_func[i] = (1/np.sqrt(2*np.pi*var))*np.exp(-1*((np.linalg.norm(q-mu[:,i]))**2)/(2*var))
            
        a = np.hstack([a_min, a_min,a_max,a_min,a_min,a_min,a_max, a_min,a_min])
        # [0.16666667  0.5        0.83333333 0.16666667 0.5 0.83333333 0.16666667 0.5        0.83333333]
        # [0.83333333  0.83333333 0.83333333 0.5        0.5 0.5        0.16666667 0.16666667 0.16666667]]
        phi = np.matmul(basis_func.T, a)

        return phi, basis_func
 

    @staticmethod
    def in_box(towers, bounding_box):
        return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                         towers[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= towers[:, 1],
                                         towers[:, 1] <= bounding_box[3]))

  
    @staticmethod
    def voronoi(towers, bounding_box, eps):
        points_left=[]
        points_right=[]
        points_down=[]
        points_up=[]
        points_center=[]
        points=[]
        # Select towers inside the bounding box
        i = agent.in_box(towers, bounding_box)
        # Mirror points
        points_center = towers[i, :]
        points_left = np.copy(points_center)
        points_left[:, 0] = bounding_box[0] - \
            (points_left[:, 0] - bounding_box[0])
        points_right = np.copy(points_center)
        points_right[:, 0] = bounding_box[1] + \
            (bounding_box[1] - points_right[:, 0])
        points_down = np.copy(points_center)
        points_down[:, 1] = bounding_box[2] - \
            (points_down[:, 1] - bounding_box[2])
        points_up = np.copy(points_center)
        points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
        points = np.append(points_center,
                        np.append(np.append(points_left,
                                            points_right,
                                            axis=0),
                                    np.append(points_down,
                                            points_up,
                                            axis=0),
                                    axis=0),
                        axis=0)
        # Compute Voronoi
        vor = scipy.spatial.Voronoi(points)
        # Filter regions
        regions = []
        veps=eps*np.array([100000,100000000,1000000000,1])
        veps[3]=0.0001
        for region in vor.regions:
            flag = True
            for index in region:
                if index == -1:
                    flag = False
                    break
                else:
                    x = vor.vertices[index, 0]
                    y = vor.vertices[index, 1]
                    # x=agent.truncc(x,5)
                    # y=agent.truncc(y,5)
                    # x=np.round(x,decimals=3)
                    # y=np.round(y,decimals=3)
                    cond1=(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                        bounding_box[2] - eps <= y and y <= bounding_box[3] + eps)
                    # cond2=(bounding_box[0] - veps[0] <= x and x <= bounding_box[1] + veps[0] and
                    #     bounding_box[2] - veps[0] <= y and y <= bounding_box[3] + veps[0])
                    # cond3=(bounding_box[0] - veps[1] <= x and x <= bounding_box[1] + veps[1] and
                    #     bounding_box[2] - veps[1] <= y and y <= bounding_box[3] + veps[1])
                    # cond4=(bounding_box[0] - veps[2] <= x and x <= bounding_box[2] + veps[2] and
                    #     bounding_box[2] - veps[2] <= y and y <= bounding_box[3] + veps[2])
                    # cond5=(bounding_box[0] - veps[3] <= x and x <= bounding_box[2] + veps[3] and
                    #      bounding_box[2] - veps[3] <= y and y <= bounding_box[3] + veps[3])
                    if not(cond1):
                        flag = False
                        # print('^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^')
                        # print(vor.vertices[index,:])
                        break
            if region != [] and flag:
                regions.append(region)
        vor.filtered_points = points_center
        vor.filtered_regions = regions
        # if len(vor.filtered_regions)<3:
            
        #     print()
        return vor
            
    @staticmethod
    def truncc(values, decs=0):
        seqr=0
        if (np.size(values))>1:
            for value in values:
                if value.all() <0.00001 or value.all() >-0.00001:
                    seqr+=1
            if seqr==np.size(values):
                return values
            else:
                return np.trunc(values*10**decs)/(10**decs)
        else:
            return np.trunc(values*10**decs)/(10**decs)

    @staticmethod
    def sorting_vertices(coords):
        # angles = np.arctan2(coords[:, 1]-np.mean(coords[:, 1]), coords[:, 0]-np.mean(coords[:, 0]))
        # angles = np.sort(angles)
        # sorty = [index[0] for index, value in np.ndenumerate(angles)]
        # coords = np.vstack([coords[i, :] for i in sorty])
        # return coords
        coords=tuple([tuple(e) for e in coords])
        center = tuple(map(operator.truediv, reduce(lambda x, y: map(operator.add, x, y), coords), [len(coords)] * 2))
        ans=sorted(coords, key=lambda coord: (-135 - math.degrees(math.atan2(*tuple(map(operator.sub, coord, center))[::-1]))) % 360)
        ans=np.array(ans,dtype=np.float32)
        return ans


    def voronoi_xy(self,xx, yy):
        seluyt=[]
        neigb_agent = []
        id=self.cfID
        temp = []
        towers=[]
        eps = 0.0001
        # bb1=xx
        # bb2=yy
        # xx=agent.truncc(xx,5)
        # xx=np.round(xx,3)
        # yy=agent.truncc(yy,5)
        # yy=np.round(yy,3)
        towers = np.vstack([xx, yy]).T
        # points = zip(xx, yy)
        # [x_min, x_max, y_min, y_max]
        bounding_box = np.array([0., 1., 0., 1.])
        vor = agent.voronoi(towers, bounding_box, eps)
        for region_1 in vor.filtered_regions:
            region_2 = vor.filtered_regions
            for i in range(len(region_2)):
                pp=Point(xx[id-1],yy[id-1])
                vermo = vor.vertices[region_2[i], :]
                epsy = 0.001
                tt=np.where(np.abs(vermo)<epsy)
                vermo[tt]=0

                sorty = agent.sorting_vertices(vermo)
                vermo=np.vstack((sorty,sorty[0,:]))
                poly=Polygon(vermo)
                if poly.contains(pp):
                    vertices_final=vermo
                    selected=i
                    seluyt=['boom']
                    break

            if region_1!= region_2[selected]:
                verty2 = vor.vertices[region_1, :]
                bamboo13=agent.truncc(vertices_final,5)
                bamboo1=np.round(bamboo13,3).tolist()
                vertices_final=np.round(bamboo13,3)

                epsy = 0.001
                tt=np.where(np.abs(verty2)<epsy)
                verty2[tt]=0

                bamboo2=agent.truncc(verty2,5)
                bamboo2=np.round(bamboo2,3)

                for k in range(len(vertices_final)-1):
                    if bamboo1[k] in bamboo2.tolist():
                        temp.append(region_1) 
            if len(temp) >= 2:
                poly=Polygon(bamboo2)
                for kk in range(len(xx)):
                    pp=Point(xx[kk],yy[kk])
                    if poly.contains(pp):
                        neigb_agent.append(kk+1)
            temp = []

        self.vertices=vertices_final
        self.neigb_agent=neigb_agent
        # print('The neibhbour agent for {} is {}'.format(neigb_agent,self.prefix))

    @staticmethod
    def sorty_square(pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect


class callback():
    def __init__(self,id,position=None,ahat=None):
        self.id=id
        self.position=position
        self.ahat=ahat
    

    def data_callback(self,data):
        msg1=data.position
        msg2=data.ahat
        self.position=np.array(msg1)
        self.ahat=np.array(msg2)


def main():


    nodeName = "crazyflies_single"
    rospy.init_node(nodeName, disable_signals=True, anonymous=False)
    # directory=os.path.join('/home/surendhar/test_ws/src/sim_cf/crazyflie_gazebo/scripts')
    nbQuads=6

    allCfDataListener = tf.TransformListener()
    cfID = rospy.get_param("~cfID")
    # ids = rospy.get_param("~id")
    Initial_Position=[0,0,0]
    cf_i=agent(cfID,Initial_Position,allCfDataListener)
    
    pathy='/home/swarmlab/research_coverage/Codes/real_time_exp_data/ie_6drones_2peaks'
    if os.path.exists(str(pathy)+'/vertices_new'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/vertices_new'+str(cfID)+'.npy')
    if os.path.exists(str(pathy)+'/positions_new'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/positions_new'+str(cfID)+'.npy')
    if os.path.exists(str(pathy)+'/ahat_new'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/ahat_new'+str(cfID)+'.npy')
    if os.path.exists(str(pathy)+'/centroid_new'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/centroid_new'+str(cfID)+'.npy')
    if os.path.exists(str(pathy)+'/true_centroid_new'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/true_centroid_new'+str(cfID)+'.npy')
    if os.path.exists(str(pathy)+'/coverage_cost'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/coverage_cost'+str(cfID)+'.npy')
    if os.path.exists(str(pathy)+'/control_input'+str(cfID)+'.npy'):
        os.remove(str(pathy)+'/control_input'+str(cfID)+'.npy')


    cf_i.cfID=cfID
    # cf_i.setParam("commander/enHighLevel", 1)
    cf_i.a_hat = np.ones(9, dtype=np.float32)*0.1
    cf_i.cap_lamda = np.zeros((9, 9), dtype=np.float32)
    cf_i.small_lamda = np.zeros((9,1), dtype=np.float32)
    cf_i.position_local=converting_short(np.array(Initial_Position)[:-1])
    cf_i.centroid=cf_i.position_local
    cf_i.unmod_goal=cf_i.position_local
    cf_i.time=0
    
    rr = 14
    rate = rospy.Rate(rr)

    
    start_time = rospy.Time.now()
    targetHeight = 0.6
    kk=0
    listnei=[]
    comm_agent=[1,2,3,4,5,6]
    for i in comm_agent:
        listnei.append(callback('cf'+str(i)))

    endcondition=False
    seq=0
    print(cf_i)
    arnold=0
    # cf_i.setParam("stabilizer/controller",1)
    cf_i.centroids=[np.array([0.8,0.4]),np.array([0.85,0.1]),np.array([0.5,0.5]),np.array([0.5,0.5]),np.array([0.85,0.1]),np.array([0.5,0.5]),np.array([0.5,0.5])]
    takeoff = 0
    while not rospy.is_shutdown():
        if not takeoff:
            cf_i.takeoff(targetHeight=0.6, duration=3.0)
            print('taking off'+str(cf_i.prefix))
            takeoff = True
            time.sleep(4)

        else:
            if endcondition:
                errorxe=targetoy-cf_i.position()
                cf_i.cmdVelocityWorld(0.5*errorxe, yawRate=0)
                end_timo=rospy.Time.now().to_sec()
                if end_timo-start_timo>7:
                    rospy.signal_shutdown('OFF')
            else:
                if seq>=1:
                    tempos=cf_i.position()
                    cf_i.position_local=converting_short(np.array([tempos[0],tempos[1]]))
                    msg=position_share()
                    msg.header.stamp=rospy.Time.now()
                    msg.header.seq+=1
                    msg.position=cf_i.position_local.tolist()
                    msg.ahat=cf_i.a_hat.tolist()
                    cf_i.pub.publish(msg)
                    xx=[]
                    yy=[]
                    ahat_dict={}
                    pos_dict={}
                    rospy.sleep(0.05)
                    for i in comm_agent:
                        prefix_nei="/cf" + str(i)
                        rospy.Subscriber(prefix_nei+'/position_ahat_pub', position_share,listnei[i-1].data_callback)
                        if not listnei[i-1].position is None:
                            xx=np.append(xx,listnei[i-1].position[0])
                            yy=np.append(yy,listnei[i-1].position[1])
                            ahat_dict.update({i:listnei[i-1].ahat})
                            pos_dict.update({i:np.array(listnei[i-1].position)})

                        else:
                            pass

                    # print('***********************************************')
                    # print('The xx values is {}'.format(xx))
                    # print('******************************************')
                    if isinstance(xx,np.ndarray) and len(xx)==nbQuads:

                        cf_i.voronoi_xy(xx,yy)

                        cf_i.find_cent()


                        cf_i.state_update()

                        cf_i.para_updatey(pos_dict,ahat_dict)

                        # cf_i.target=np.append(converting_long(np.array(cf_i.unmod_goal,dtype=np.float32)),targetHeight)
                        posion=cf_i.position()
                        posion=posion[-1:]
                        errorxp=cf_i.control_gain*(targetHeight-posion)
                        targeton=np.append(0.4*(cf_i.control_input),errorxp)
                        cf_i.cmdVelocityWorld(targeton, yawRate=0)
                        print('==============================')
                        print(cf_i.cfID)
                        print(targeton)
                        print(converting_long(cf_i.centroid)-(cf_i.position()[:-1]))
                        print(cf_i.centroid-converting_short(cf_i.position()[:-1]))
                        print('==============================')
                        # print('***********************************')
                        # print('done')
                        # print('********************************')
                        # if (seq%20)==0:
                        #     print('*********************************')
                        #     print('*********************************')
                        #     print('The updated targetfor {} value is {}'.format(cf_i.prefix,targeton))
                        #     print('*********************************')
                        #     print('*********************************')
                        if (seq%10)==0:
                            print('*********************************')
                            print('*********************************')
                            print('The updated a_hat paramater for {} value is {}'.format(cf_i.prefix,cf_i.a_hat))
                            print('The updated centroid paramater for {} value is {}'.format(cf_i.prefix,cf_i.centroid))
                            print('*********************************')
                            print('*********************************')
                        
                        with open(str(pathy)+'/vertices_new'+str(cfID)+'.npy', 'ab') as fff:
                            np.save(fff, cf_i.vertices)
                        with open(str(pathy)+'/positions_new'+str(cfID)+'.npy', 'ab') as fpp:
                            np.save(fpp, (np.array(cf_i.position_local,dtype=np.float32)))
                        with open(str(pathy)+'/centroid_new'+str(cfID)+'.npy', 'ab') as fppc:
                            np.save(fppc, (np.array(cf_i.centroid,dtype=np.float32))) 
                        with open(str(pathy)+'/true_centroid_new'+str(cfID)+'.npy', 'ab') as fppc11:
                            np.save(fppc11, (np.array(cf_i.true_centroid,dtype=np.float32))) 
                        with open(str(pathy)+'/ahat_new'+str(cfID)+'.npy', 'ab') as fhat:
                            np.save(fhat, cf_i.a_hat) 
                        with open(str(pathy)+'/iteration_new'+str(cfID)+'.npy', 'ab') as itm:
                            np.save(itm, seq) 
                        with open(str(pathy)+'/coverage_cost'+str(cfID)+'.npy', 'ab') as itm_bal:
                            np.save(itm_bal, cf_i.coverage_cost) 
                        with open(str(pathy)+'/control_input'+str(cfID)+'.npy', 'ab') as itm_bal1:
                            np.save(itm_bal1, cf_i.control_input) 
                        rospy.sleep(0.001)

                        cf_i.time=cf_i.time+cf_i.stepsize
                        
                    else:
                        pass                   
                else:
                    pass
        seq+=1
        print('The iteration of the loop is for the agent {} {}'.format(cf_i.prefix,seq))
        rate.sleep()
        with open('iteration_new'+str(cfID)+'.npy', 'wb') as itm:
            np.save(itm, seq) 
        if seq>140:
            endcondition=True
            targetoy=cf_i.position()[:-1]
            targetoy=np.append(targetoy,0.05)
            start_timo=rospy.Time.now().to_sec()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception:
        rospy.signal_shutdown('OFF')