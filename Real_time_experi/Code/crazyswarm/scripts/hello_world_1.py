#!/usr/bin/env python3
import sys
import warnings
import tf2_ros
import time
import yaml
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
# from pycrazyswarm import Crazyflie, TimeHelper
from tf2_msgs.msg import TFMessage

# from crazyswarm.msg import  FullState, Position, VelocityWorld
# from crazyswarm.msg import Position,GenericLogData
    # def local_position_callback(self,msg):
    #     index=msg.name.index('cf'+str(self.cfID))
    #     temp=(msg.pose)[index]
    #     self.position_local=converting_short(np.array([temp.position.x,temp.position.y]))

def local_position_callback(msg):
    for tfg in msg.transforms:
        if(tfg.child_frame_id=='cf1'):
            rospy.loginfo(tfg.transform.translation.x)
    return msg


def main():
    # Initialize node crazyflie
    # -----------------------------------------------
    # -----------------------------------------------
    nodeName = "crazyflie_single"                                                                         
    rospy.init_node(nodeName,disable_signals=True)
    r = 10
    rate = rospy.Rate(r)
    # Define parameters required for operation
    # -----------------------------------------------
    cfID = 1
    tf_buffer = tf2_ros.Buffer()
    tf_listen=tf2_ros.TransformListener(tf_buffer)

    target_location=[np.array([0.7,0.7,0.7]),np.array([0.7,-0.7,0.7]),np.array([-0.7,-0.7,0.7]),np.array([-0.7,0.7,0.7])]
    takeoff=0
    current_time=rospy.Time.now().to_sec()
    land=0
    kk=0.1
    print(current_time)

    while not rospy.is_shutdown():
        if not takeoff:
            ggf=rospy.Subscriber('/tf' ,TFMessage, local_position_callback)
            # print(ggf)
            takeoff=False
        else:
            # CF.land(targetHeight=0.003, duration=2)
            time.sleep(0.5)
            rospy.signal_shutdown('OFF')
        rate.sleep()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
