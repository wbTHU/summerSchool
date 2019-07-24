import vrep
import numpy as np
import math
import time
import ctypes
import cv2
import VisionSensor
from matplotlib import pyplot as plt


Sensor = VisionSensor.VisionSensor

tstep = 0.005
baseName = 'Quadricopter_base'
targetName = 'Quadricopter_target'
landName = 'land_plane'
zedName = 'zed_vision'
zedNum = 2
xAngle = 85
row = 720
col = 1280
limit = 50


def int2uint8(num):
    if num < 0:
        return num + 2 ** 8
    else:
        return num


def ax2pos(leftZed, rightZed, ori):
    baseline = abs(leftZed.pos[1] - rightZed.pos[1])
    baseline = 0.12

    theta = ori[2]

    x = leftZed.pos[0]
    y = leftZed.pos[1]
    z = leftZed.pos[2]

    dx = abs(leftZed.targetX - rightZed.targetX)

    alpha = 85 * math.pi / 180

    
    tz = baseline / (dx * math.tan(alpha / 2) / 640)
    # tx = baseline * ( leftZed.targetX - 640) / dx
    tx = math.tan(alpha / 2) * tz / 640 * (leftZed.targetX - 640)
    ty = math.tan(alpha / 2) * tz / 640 * (leftZed.targetY - 360)

    # _x = x - ty
    # _y = y - tx
    # _z = z - tz

    _x = x + (tx * math.sin(theta) + tz * math.cos(theta)) # 0: +tz; 90: +tx; 180: -tz; -90: -tx
    _y = y - (tx * math.cos(theta) - tz * math.sin(theta)) # 0: -tx; 90: +tz; 180: +tx; -90: -tz
    _z = z - ty

    res = []
    res.append(_x)
    res.append(_y)
    res.append(_z)

    return res


print ('program started!')
vrep.simxFinish(-1)

while True:
    clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
    if clientID != -1:
        break
    else:
        print ('Cannot connect to server!')
        time.sleep(0.05)

print ('Connection Success!')


vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot)
vrep.simxSynchronous(clientID,True)
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)




_, landHandle = vrep.simxGetObjectHandle(clientID,landName,vrep.simx_opmode_blocking)
    
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, targetHandle = vrep.simxGetObjectHandle(clientID,targetName,vrep.simx_opmode_blocking)


print('Handles available!')

vrep.simxSynchronousTrigger(clientID)



