import vrep
import numpy as np
import math
import time
import ctypes
import cv2




tstep = 0.005
baseName = 'Quadricopter_base'
targetName = 'Quadricopter_target'
landName = 'land_plane'
zedName = 'zed_vision'
zedNum = 2
xAngle = 85


def int2uint8(num):
    if num < 0:
        return num + 2 ** 8
    else:
        return num


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

zedHandle = np.zeros((zedNum,),dtype=np.int)

for i in range(zedNum):
    zN = zedName + str(i)
    _, zedHandle[i] = vrep.simxGetObjectHandle(clientID,zN,vrep.simx_opmode_blocking)


_, landHandle = vrep.simxGetObjectHandle(clientID,landName,vrep.simx_opmode_blocking)
    
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, targetHandle = vrep.simxGetObjectHandle(clientID,targetName,vrep.simx_opmode_blocking)

print('Handles available!')



vrep.simxSynchronousTrigger(clientID)

_, originalP = vrep.simxGetObjectPosition(clientID,targetHandle,-1,vrep.simx_opmode_blocking)
_, landP = vrep.simxGetObjectPosition(clientID,landHandle,-1,vrep.simx_opmode_blocking)
pos = originalP

_, zedPos = vrep.simxGetObjectPosition(clientID,zedHandle[0],-1,vrep.simx_opmode_blocking)


vision = []
trans = []
for i in range(zedNum):
    _, sensor, v = vrep.simxGetVisionSensorImage(clientID,zedHandle[i],0,vrep.simx_opmode_blocking)
    vision.append(v)

for v in vision:
    ts = []
    for i in v:
        t = int2uint8(i)
        ts.append(t)
    trans.append(ts)

vv = trans[0]


row = 720
col = 1280
re = []
for i in range(row):
    t = []
    for j in range(col):
        s = []
        for k in range(3):
            m = vv[i * col * 3 + j * 3 + k]
            s.append(m)
        t.append(s)
    re.append(t)
print(len(re))

# f = open('00.txt', 'w')
# f.write(str(re))
# f.close()

mat = np.array(re,dtype=np.uint8)
mat1 = cv2.flip(mat,0,dst=None) #垂直镜像

# cv2.imshow('hhh',mat1)
# cv2.waitKey()

gray = cv2.cvtColor(mat1,cv2.COLOR_BGR2GRAY) #进行灰度转化

limit = 220
mask = cv2.inRange(gray,0,limit) #将0~220范围内的像素点全部转化为白色，其余为黑色，达到凸显二维码的目的
# cv2.imshow('hhh',mask)
# cv2.waitKey()

# 利用黑色像素的位置得到二维码的中心坐标
maxx=0
minx=100000
maxy=0
miny=100000
xlen = 1280
ylen = 720

for i in range(xlen): 
    for j in range(ylen):
        if mask[j][i] > 0:
            maxx=max(maxx,i)
            minx=min(minx,i)
            maxy=max(maxy,j)
            miny=min(miny,j)
targetX = (maxx + minx) / 2
targetY = (maxy + miny) / 2









