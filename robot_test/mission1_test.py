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

def ax2pos(zedpos,xangle,x,y): # 注意mission01飞机朝向 x,y指的是在图片的坐标系内像素点的坐标
    z = abs(zedpos[2])
    d = math.tan(xangle * math.pi / 360) * z / 640
    # _x = zedpos[0] - (x - 640) * d
    # _y = zedpos[1] - (y - 360) * d
    _x = zedpos[0] - (y - 360) * d
    _y = zedpos[1] - (x - 640) * d
    res = []
    res.append(_x)
    res.append(_y)
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

zedHandle = np.zeros((zedNum,),dtype=np.int)

for i in range(zedNum):
    zN = zedName + str(i)
    _, zedHandle[i] = vrep.simxGetObjectHandle(clientID,zN,vrep.simx_opmode_blocking)


_, landHandle = vrep.simxGetObjectHandle(clientID,landName,vrep.simx_opmode_blocking)
    
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, targetHandle = vrep.simxGetObjectHandle(clientID,targetName,vrep.simx_opmode_blocking)

_, sensorHandle = vrep.simxGetObjectHandle(clientID, sensorName, vrep.simx_opmode_blocking)



print('Handles available!')

vrep.simxSynchronousTrigger(clientID)

_, originalP = vrep.simxGetObjectPosition(clientID,targetHandle,-1,vrep.simx_opmode_blocking)
_, landP = vrep.simxGetObjectPosition(clientID,landHandle,-1,vrep.simx_opmode_blocking)
pos = originalP





vrep.simxSynchronousTrigger(clientID)
    
# 利用image.py中的方法找二维码（可视区域）中心点
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

zedPos = []

for i in range(zedNum):
    _, zP = vrep.simxGetObjectPosition(clientID, zedHandle[i],-1,vrep.simx_opmode_blocking)
    zedPos.append(zP)


masks = []

for v in trans:
    re = []
    for i in range(row):
        t = []
        for j in range(col):
            s = []
            for k in range(3):
                m = v[i * col * 3 + j * 3 + k]
                s.append(m)
            t.append(s)
        re.append(t)
    mat = np.array(re,dtype=np.uint8)
    mat1 = cv2.flip(mat,0,dst=None) #垂直镜像
    
    gray = cv2.cvtColor(mat1,cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0）,达到凸显二维码的目的
    # cv2.imshow('hhh',mask)
    # cv2.waitKey()
    masks.append(mask)

targetX = 0
targetY = 0

tx = []
ty = []

for q in range(zedNum):
    mask = masks[q]

    # 先找到白色大块边界（去除边界黑块影响）
    # 利用黑色像素的位置得到二维码的中心坐标

    MAXX=0
    MINX=100000
    MAXY=0
    MINY=100000
    xlen = 1280
    ylen = 720

    for i in range(xlen): 
        for j in range(ylen):
            if mask[j][i] > 0: # 找白色
                MAXX=max(MAXX,i)
                MINX=min(MINX,i)
                MAXY=max(MAXY,j)
                MINY=min(MINY,j)
                # 得到白色边界

    maxx=0
    minx=100000
    maxy=0
    miny=100000
    xlen = 1280
    ylen = 720

    for i in range(xlen): 
        for j in range(ylen):
            if mask[j][i] == 0 and i >= MINX and i <= MAXX and j >= MINY and j <= MAXY: # 找黑色
                maxx=max(maxx,i)
                minx=min(minx,i)
                maxy=max(maxy,j)
                miny=min(miny,j)
    center_x = (maxx+minx)/2
    center_y = (maxy+miny)/2

    print(center_x)
    print(center_y)

    res = ax2pos(zedPos[q],xAngle,center_x,center_y)
    tx.append(res[0])
    ty.append(res[1])

    # tx.append(center_x)
    # ty.append(center_y)
              
targetX = (tx[0] + tx[1]) / 2
targetY = (ty[0] + ty[1]) / 2

print(targetX)
print(targetY)


# _,nowPos = vrep.simxGetObjectPosition(clientID,baseHandle,-1,vrep.simx_opmode_blocking)
# print('nowPos' + str(nowPos))
# z = nowPos[2]
# print('z is' + str(z))
# res = ax2pos(nowPos,xAngle,targetX,targetY)
# _x = res[0]
# _y = res[1]


_x = targetX
_y = targetY


print('_x is'+ str(_x))
print('_y is'+ str(_y)) 



if vrep.simxGetConnectionId(clientID) != -1:

    # 运动
    # if abs(landP[2] - pos[2]) < 0.2:
    #     break
    # pos[2] = pos[2] - 0.01
    # vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_oneshot)

    
    # vrep.simxPauseCommunication(clientID, True)

    time.sleep(0.1)
    print('start to go back')


    if pos[0] > _x:
        Xsign = -1
    else: 
        Xsign = 1

    if pos[1] > _y:
        Ysign = -1
    else: 
        Ysign = 1

    while abs(pos[0] - _x) > 0.05:
        pos[0] = pos[0] + Xsign * 0.005
        vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)

    while abs(pos[1] - _y) > 0.05:
        pos[1] = pos[1] + Ysign * 0.005
        vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)

    print('end?')
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

    print('land')
    
    k = vrep.simxReadProximitySensor(clientID,sensorHandle,vrep.simx_opmode_blocking)
    isDetected = k[1]
    
    while( isDetected == False):
        pos[2] = pos[0] -  0.005
        vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
        k = vrep.simxReadProximitySensor(clientID,sensorHandle,vrep.simx_opmode_blocking)
        isDetected = k[1]
    
    print('land success!')

    while(True):
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)



