#!/usr/bin/env python3
import sys
import warnings
import tf
import time
import yaml
import rospy
import numpy as np
from pycrazyswarm import Crazyflie, TimeHelper
# from crazyswarm.msg import  FullState, Position, VelocityWorld
from crazyswarm.msg import Position,GenericLogData


def main():
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
    target_location=[np.array([0.7,0.7,0.7]),np.array([0.7,-0.7,0.7]),np.array([-0.7,-0.7,0.7]),np.array([-0.7,0.7,0.7])]
    takeoff=0
    current_time=rospy.Time.now().to_sec()
    land=0
    kk=0.1
    waypointt=False
    print(current_time)
    while not rospy.is_shutdown():
        if not takeoff:
            CF.takeoff(targetHeight=0.3, duration=1.5)
            # posi=CF.position()
            # print(posi)
            time.sleep(8)
            takeoff=True
        else:
            CF.land(targetHeight=0.003, duration=2.0)
            time.sleep(3.5)
            rospy.signal_shutdown('OFF')
        rate.sleep()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass