#!/usr/bin/env python3

import numpy as np
from pycrazyswarm import *

target_location_1=[np.array([1.0,1.0,0.5]),np.array([1.0,-1.0,0.5]),np.array([-1.0,-1.0,0.5]),np.array([-1.0,1.0,0.5])]
target_location=[np.array([0.5,0.5,0.5]),np.array([0.5,-0.5,0.5]),np.array([-0.5,-0.5,0.5]),np.array([-0.5,0.5,0.5])]

drone_relative_positions = [
        np.array([0.5, 0.5, 0]),  
        np.array([0.5, -0.5, 0]),  
        np.array([-0.5, -0.5, 0]), 
        np.array([-0.5, 0.5, 0])  
    ]
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=0.5, duration=4)
    timeHelper.sleep(10)

    # ii=0
    # for cf in allcfs.crazyflies:
    #     pos=target_location_1[ii]
    #     cf.goTo(pos, 0, 2.5)
    #     # timeHelper.sleep(4)
    #     ii+=1
    # timeHelper.sleep(7)

    # for cf in allcfs.crazyflies:
    #     pos=drone_relative_positions[ii]
    #     cf.goTo(pos, 0, 2.5)
    #     # timeHelper.sleep(4)
    #     ii+=1
    # timeHelper.sleep(7)

    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(4)
