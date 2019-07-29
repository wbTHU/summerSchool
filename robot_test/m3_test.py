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




def tpos(zedPos, dx, axis, zori, ori):  # zori 为相机   ori为整体
    baseline = 0.12

    theta = ori[2]

    x = zedPos[0]
    y = zedPos[1]
    z = zedPos[2]

    alpha = 85 * math.pi / 180

    a = math.pi -  zori[0]
    # b = zori[1]
    # c = zori[2]


    
    tz = baseline / (dx * math.tan(alpha / 2) / 640)
    # tx = baseline * ( leftZed.targetX - 640) / dx
    tx = math.tan(alpha / 2) * tz / 640 * (axis[0] - 640)
    ty = math.tan(alpha / 2) * tz / 640 * (axis[1] - 360)

    print(str([tx, ty, tz]))

    l = math.sqrt(tz ** 2 + ty ** 2)

    m = math.atan(ty / tz)

    tt = a - m

    print(l)
    print(tt)

    tz = l * math.cos(tt)

    ty = l * math.sin(tt)


    _x = x + tx * math.sin(theta) - ty * math.cos(theta) # 0: -ty; 90: +tx; 180: +ty; -90: -tx
    _y = y - tx * math.cos(theta) - ty * math.sin(theta) # 0: -tx; 90: -ty; 180: +tx; -90: +ty
    _z = z - tz

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

lZed = Sensor(clientID,'zed_vision1')

rZed = Sensor(clientID,'zed_vision0')

_, zori = vrep.simxGetObjectOrientation(clientID, lZed.zedHandle, -1, vrep.simx_opmode_blocking)

print(zori)

_, ori = vrep.simxGetObjectOrientation(clientID, baseHandle, -1, vrep.simx_opmode_blocking)

print(ori)

lZed.findTarget()

rZed.findTarget()

dx = abs( lZed.targetX - rZed.targetX )
print(dx)

axis = [lZed.targetX, lZed.targetY ]

r = tpos(lZed.pos,dx,axis, zori,ori )

print(r)

# ll = lZed.image
# rr = rZed.image














# ll = cv2.cvtColor(ll,cv2.COLOR_RGB2GRAY)
# rr = cv2.cvtColor(rr,cv2.COLOR_RGB2GRAY)





# blockSize = 48
# stereo = cv2.StereoSGBM_create(minDisparity=1,
#              numDisparities=16,
#              blockSize=11,
#              uniquenessRatio = 5,
#              speckleWindowSize = 100,
#              speckleRange = 1,
#              disp12MaxDiff = 200,
#              P1 = 8*3*blockSize**2,
#              P2 = 32*3*blockSize**2)
# disparity = stereo.compute(ll,rr)

# plt.imshow(disparity,'gray')
# plt.show()

# lzPos = lZed.pos

# _, ori = vrep.simxGetObjectOrientation(clientID, baseHandle, -1, vrep.simx_opmode_blocking)
# print(ori)

# dx = abs(disparity[200][800] ) / 16

# print(dx)


# a = [800, 200]
# res = tpos(lzPos, dx, a, zori, ori)

# print(res)

vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)



