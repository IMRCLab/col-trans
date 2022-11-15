#!/usr/bin/env python

import numpy as np

from py_crazyswarm2 import *
import subprocess 
import shutil
import os
import time

# SL
Ids =  [2, 6]
Heights = [0.3, 1.2]
Radius = 0.3

cf_config = {
    2: {
        'waypoints': [
            [1.0,0.5,0.5],
            [-0.5,-1.0,0.5],
            [1.0,0.5,0.5],
            [-0.5,-1.0,0.5],
            [0.5,-0.5,0.5],
        ]
    },
    6: {
        'waypoints': [
            [-0.5,-1.0,0.6],
            [1.0,0.5,0.6],
            [-0.5,-1.0,0.6],
            [1.0,0.5,0.6],
            [0.0,-0.5,0.6],
        ]
    }
}

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # print(list(allcfs.crazyfliesById.keys()))
    # exit()

    # for cf in allcfs.crazyflies:
    #     cf.setParam("ctrlLee/Kw_x", 10)

    # allcfs.setParam("satbilizer/controller", 5) # use Lee controller

    allcfs.takeoff(targetHeight=np.min(Heights), duration=3.0)
    timeHelper.sleep(3.5)

    # # go to initial positions
    # angles = np.linspace(0, 2*np.pi, 2 * len(Ids), endpoint=False)
    # # print(angles, angles[0:len(Ids)], angles[len(Ids):])
    # for angle, cfid, height in zip(angles[0:len(Ids)], Ids, Heights):
    #     pos = np.array([np.sin(angle) * Radius, np.cos(angle) * Radius, height])
    #     allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)

    # go to initial positions
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 0.5])
        cf.goTo(pos, 0, 3.0)

    timeHelper.sleep(5)

    for cf in allcfs.crazyflies:
        print(cf, ' logging')
        cf.setParam("usd/logging", 1)

    for i in range(5):
        for cfid in Ids:
            pos = np.array(cf_config[cfid]['waypoints'][i])
            print(cfid, pos)
            allcfs.crazyfliesById[cfid].goTo(pos, 0, 5.0)
        timeHelper.sleep(5.5)

    # for swapTime in SwapTimes:
    #     # swap 1
    #     for angle, cfid, height in zip(angles[len(Ids):], Ids, Heights):
    #         pos = np.array([np.sin(angle) * Radius, np.cos(angle) * Radius, height])
    #         allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
    #     timeHelper.sleep(swapTime+1.5)

    #     # swap 2
    #     for angle, cfid, height in zip(angles[0:len(Ids)], Ids, Heights):
    #         pos = np.array([np.sin(angle) * Radius, np.cos(angle) * Radius, height])
    #         allcfs.crazyfliesById[cfid].goTo(pos, 0, 3.0)
    #     timeHelper.sleep(swapTime+1.5)

    # allcfs.setParam("usd/logging", 0)
    for cf in allcfs.crazyflies:
        print(cf,' unlogging')
        cf.setParam("usd/logging", 0)

    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, np.min(Heights)])
        cf.goTo(pos, 0, 3.0)
    timeHelper.sleep(5)

    # timeHelper.sleep(10)

    # timeHelper.sleep(5)
   
    allcfs.land(targetHeight=0.02, duration=3.0)
    timeHelper.sleep(5.0)

if __name__ == "__main__":
    main()