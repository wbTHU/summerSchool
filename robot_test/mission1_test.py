import vrep
import numpy as np
import math
import time
import ctypes
import cv2
import VisionSensor
import Hand
from matplotlib import pyplot as plt


Sensor = VisionSensor.VisionSensor
Hand = Hand.Hand

tstep = 0.005
baseName = 'Quadricopter_base'
targetName = 'Quadricopter_target'
landName = 'land_plane'
sensorName = 'land_detecter1'
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


def ax2pos(leftZed, rightZed):
    baseline = abs(leftZed.pos[1] - rightZed.pos[1])
    x = leftZed.pos[0]
    y = leftZed.pos[1]
    z = leftZed.pos[2]

    dx = abs(leftZed.targetX - rightZed.targetX)

    alpha = 85 * math.pi / 180

    tx = baseline * ( leftZed.targetX - 640) / dx
    ty = math.tan(alpha / 2) * z / 640 * (leftZed.targetY - 360)
    tz = baseline / (dx * math.tan(alpha / 2) / 640)

    _x = x - ty
    _y = y - tx
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

# zedHandle = np.zeros((zedNum,),dtype=np.int)

# for i in range(zedNum):
#     zN = zedName + str(i)
#     _, zedHandle[i] = vrep.simxGetObjectHandle(clientID,zN,vrep.simx_opmode_blocking)


_, landHandle = vrep.simxGetObjectHandle(clientID,landName,vrep.simx_opmode_blocking)
    
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, targetHandle = vrep.simxGetObjectHandle(clientID,targetName,vrep.simx_opmode_blocking)

_, sensorHandle = vrep.simxGetObjectHandle(clientID, sensorName, vrep.simx_opmode_blocking)



print('Handles available!')

vrep.simxSynchronousTrigger(clientID)

_, originalP = vrep.simxGetObjectPosition(clientID,targetHandle,-1,vrep.simx_opmode_blocking)
_, landP = vrep.simxGetObjectPosition(clientID,landHandle,-1,vrep.simx_opmode_blocking)
pos = originalP


# h = Hand(clientID,'goto_close')


# vrep.simxSynchronousTrigger(clientID)

# h.close()
# vrep.simxSynchronousTrigger(clientID)


_, ori = vrep.simxGetObjectOrientation(clientID,baseHandle,-1,vrep.simx_opmode_blocking)
print(ori)
zd0 = Sensor(clientID,zedName+'0')
zd1 = Sensor(clientID,zedName+'1')

baseline = abs(zd0.pos[1] - zd1.pos[1])
print(baseline)
zd0.findTarget()
zd1.findTarget()

x = zd0.pos[0]
y = zd0.pos[1]
z = zd0.pos[2]
dx = abs(zd0.targetX - zd1.targetX)
print(dx)

alpha = 85 * math.pi / 180

tx = baseline * ( zd0.targetX - 640) / dx
_y = y - tx

_z = z - baseline / (dx * math.tan(alpha / 2) / 640)

ty = math.tan(alpha / 2) * z / 640 * (zd0.targetY - 360)
_x = x - ty

print(_x)
print(_y)
print(_z)





# r = zd0.findTarget()
# print(r)
# print(zd0.targetX)
# print(zd0.targetY)





# zeds = []
# for i in range(zedNum):
#     zN = zedName + str(i)
#     z = Sensor(clientID,zN)
#     zeds.append(z)

# # zeds[0].findBlack()
# zeds[0].findQR()
# # zeds[0].findTarget()

# # zeds[1].findCircle()

# cv2.imwrite('I.png',zeds[0].image)

# Df = 5e3
# Dm = 5e-3
# xA = xAngle * math.pi / 180



# depthMap = zeds[0].depthMap



# z = zeds[0].pos[2]

# _x = zeds[0].targetX
# _y = zeds[0].targetY
# print('_x is ' + str(_x))
# print('_y is ' + str(_y))


# dep = Dm +   (Df - Dm )* depthMap[int(_y)][int(_x)]

# print(z - dep)

# res = ax2pos(zeds[0].pos,xAngle,_x,_y)
# x = res[0]
# y = res[1]
# print('x is ' + str(x))
# print('y is ' + str(y))

# zeds[1].findQR()
# res1 = ax2pos(zeds[1].pos,xAngle,zeds[1].targetX,zeds[1].targetY)
# xx = (res[0] + res1[0]) / 2
# yy = (res[1] + res1[1]) / 2
# print('xx is ' + str(xx))
# print('yy is ' + str(yy))

# lm = cv2.cvtColor(zeds[0].image,cv2.COLOR_BGR2GRAY)
# rm = cv2.cvtColor(zeds[1].image,cv2.COLOR_BGR2GRAY)
# stereo = cv2.StereoSGBM_create(1,16,5)
# disparity = stereo.compute(lm,rm)
# plt.imshow(disparity,'gray')
# plt.show()


# _,nowPos = vrep.simxGetObjectPosition(clientID,baseHandle,-1,vrep.simx_opmode_blocking)
# print('nowPos' + str(nowPos))
# z = nowPos[2]
# print('z is' + str(z))
# res = ax2pos(nowPos,xAngle,targetX,targetY)
# _x = res[0]
# _y = res[1]


# _x = targetX
# _y = targetY


# print('_x is '+ str(_x))
# print('_y is '+ str(_y)) 

# vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot)
# vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)


# if vrep.simxGetConnectionId(clientID) != -1:

    # 运动
    # if abs(landP[2] - pos[2]) < 0.2:
    #     break
    # pos[2] = pos[2] - 0.01
    # vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_oneshot)

    
    # vrep.simxPauseCommunication(clientID, True)

    # print('start to go back')


    # if pos[0] > _x:
    #     Xsign = -1
    # else: 
    #     Xsign = 1

    # if pos[1] > _y:
    #     Ysign = -1
    # else: 
    #     Ysign = 1

    # while abs(pos[0] - _x) > 0.05:
    #     pos[0] = pos[0] + Xsign * 0.005
    #     vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    #     vrep.simxSynchronousTrigger(clientID)
    #     vrep.simxGetPingTime(clientID)

    # while abs(pos[1] - _y) > 0.05:
    #     pos[1] = pos[1] + Ysign * 0.005
    #     vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    #     vrep.simxSynchronousTrigger(clientID)
    #     vrep.simxGetPingTime(clientID)

    # print('end?')
    # vrep.simxSynchronousTrigger(clientID)
    # vrep.simxGetPingTime(clientID)

    # print('land')
    
    # k = vrep.simxReadProximitySensor(clientID,sensorHandle,vrep.simx_opmode_blocking)
    # isDetected = k[1]
    
    # while( isDetected != True):
    #     pos[2] = pos[2] -  0.005
    #     vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    #     vrep.simxSynchronousTrigger(clientID)
    #     vrep.simxGetPingTime(clientID)
    #     k = vrep.simxReadProximitySensor(clientID,sensorHandle,vrep.simx_opmode_blocking)
    #     isDetected = k[1]
    #     print(isDetected)
    
    # rs = 0.2

    # while( rs > 0):
    #     pos[2] = pos[2] -  0.005
    #     vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    #     vrep.simxSynchronousTrigger(clientID)
    #     vrep.simxGetPingTime(clientID)
    #     rs = rs - 0.005

    
    # print('land success!')

    # vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot)



