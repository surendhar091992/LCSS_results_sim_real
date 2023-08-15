#!/usr/bin/env python3
import sys
import pickle
import warnings
import tf
import time
import yaml
import rospy
import numpy as np
from pycrazyswarm import Crazyflie, TimeHelper
# from crazyswarm.msg import  FullState, Position, VelocityWorld
from crazyswarm.msg import Position,GenericLogData


class callback():
    def __init__(self,sen_val=None):
        self.sen_val=sen_val
    
    def data_callback(self,data):
        msg1=data.values[0]
        self.sen_val=msg1

def main():
    pathy='/home/swarmlab/real_time_exp_data'
    # Initialize node crazyflie
    # -----------------------------------------------
    # -----------------------------------------------
    nodeName = "crazyflie"                                                                         
    rospy.init_node(nodeName,disable_signals=True)
    r = 10
    rate = rospy.Rate(r)
    # Define parameters required for operation
    # -----------------------------------------------
    cfID = rospy.get_param("~cfID")
    ownFrame = "/"+"cf"+str(cfID)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    # -----------------------------------------------

    initialPosition = [0,0,0]
    
    

    # For accessing data of all crayflies
    # -----------------------------------------------
    allCfDataListener = tf.TransformListener()
    # -----------------------------------------------

    
    # Initialize crazyflie
    # -----------------------------------------------
    CF = Crazyflie(cfID, initialPosition, allCfDataListener)
    # -----------------------------------------------
    # print(initialPosition)
    target_location=[np.array([0.7,0.7,0.5]),np.array([0.7,-0.7,0.5]),np.array([-0.7,-0.7,0.5]),np.array([-0.7,0.7,0.5])]
    takeoff=0
    current_time=rospy.Time.now().to_sec()
    targetHeight=0.6
    land=0
    kk=1
    points=[]
    data={}
    minX, maxX, minY, maxY = -2, 2, -2, 2
    xx = np.linspace(minX, maxX, 30)
    yy = np.linspace(minY, maxY, 30)
    waypointt=False
    cf3=callback()
    while not rospy.is_shutdown():
        if not takeoff:
            CF.takeoff(targetHeight, duration=2.0)
            posi=CF.position()
            time.sleep(5)
            takeoff=True
            waypointt=True
        elif waypointt:
            for row in xx:
                if((kk%2)!=0):
                    for col in yy:
                        rospy.sleep(0.05)
                        rospy.Subscriber('/cf3'+'/log2', GenericLogData,cf3.data_callback)
                        rospy.sleep(0.05)
                        points.append([row, col])
                        temp=tuple([row,col])
                        data.update({temp:cf3.sen_val})
                        targeton=np.append(np.array(temp),targetHeight)
                else:
                    for col in np.flip(yy):
                        rospy.sleep(0.05)
                        rospy.Subscriber('/cf3'+'/log2', GenericLogData,cf3.data_callback)
                        rospy.sleep(0.05)
                        points.append([row, col])
                        temp=tuple([row,col])
                        data.update({temp:cf3.sen_val})
                        targeton=np.append(np.array(temp),targetHeight)

                posion=CF.position()
                errorxp=targeton-posion
                CF.cmdVelocityWorld(1.2*errorxp, yawRate=0)
                time.sleep(0.01)
                kk+=1
            waypointt=False
        else:
            CF.land(targetHeight=0.04, duration=3)
            time.sleep(0.5)
            rospy.signal_shutdown('OFF')
        rate.sleep()
    with open(str(pathy)+'data_light.pkl', 'wb') as fp:
        pickle.dump(data, fp)
        print('Data has been saved successfully to file')
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass