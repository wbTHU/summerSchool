import vrep
import numpy as np
import math
import time
import ctypes
import cv2

tstep = 0.005
baseName = 'Quadricopter_base'
moveName = 'Quadricopter_target'
targetName = 'Target'
handName = 'JacoHand'
handSignal = 'close_hand'
zedName = 'zed_vision'
zedNum = 2
xAngle = 85
row = 720
col = 1280

def int2uint8(num):
    if num < 0:
        return num + 2 ** 8
    else:
        return num


def ax2pos(zedpos,xangle,x,y):
    z = zedpos[2]
    d = math.tan(xangle * math.pi / 360) * z / 640
    _x = zedpos[0] - (x - 640) * d
    _y = zedpos[1] - (y - 360) * d
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
    
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, moveHandle = vrep.simxGetObjectHandle(clientID, moveName, vrep.simx_opmode_blocking)
_, targetHandle = vrep.simxGetObjectHandle(clientID,targetName,vrep.simx_opmode_blocking)
err, scriptHandle = vrep.simxGetObjectHandle(clientID,'util_funcs',vrep.simx_opmode_blocking)

print('Handles available!')

vrep.simxSynchronousTrigger(clientID)

# _, targetPos = vrep.simxGetObjectPosition(clientID,targetHandle,-1,vrep.simx_opmode_blocking)

_, _, targetPos, _, _ = vrep.simxCallScriptFunction(clientID,'my_funcs',vrep.sim_scripttype_customizationscript,'my_get_target_platform_pos',[],[],[],'',vrep.simx_opmode_blocking)
_, _, endPos, _, _ = vrep.simxCallScriptFunction(clientID,'my_funcs',vrep.sim_scripttype_customizationscript,'my_get_end_point_pos',[],[],[],'',vrep.simx_opmode_blocking)

_, originalPos = vrep.simxGetObjectPosition(clientID,moveHandle,-1,vrep.simx_opmode_blocking)

print(str(targetPos))
print(str(endPos))

pos = originalPos


x = targetPos[0]
y = targetPos[1]



if pos[0] > x:
    Xsign = -1
else: 
    Xsign = 1

if pos[1] > y:
    Ysign = -1
else: 
    Ysign = 1



# print('start!')



# while abs(pos[0] - x) > 0.05:
#     pos[0] = pos[0] + Xsign * 0.01
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)


# while abs(pos[1] - y) > 0.05:
#     pos[1] = pos[1] + Ysign * 0.01
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)


# print('end!')


# 利用image.py中的方法找T（可视区域）中心点（可以写成函数方便调用）
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

masks = []
tx = []
ty = []
r = 0
zedPos = []

for i in range(zedNum):
    _, zP = vrep.simxGetObjectPosition(clientID, zedHandle[i],-1,vrep.simx_opmode_blocking)
    print(zP)
    zedPos.append(zP)

for q in range(zedNum):
    v = trans[q]
    re = []
    for i in range(row):
        t = []
        for j in range(col):
            s = []
            for k in range(3):
                m = v[i * col * 3 + j * 3 + k]
                s.append(m)
            s[0], s[2] = s[2], s[0]  # opencv使用的是BGR，因此需要交换G和B
            t.append(s)
        re.append(t)
    
    mat = np.array(re,dtype=np.uint8)
    mat1 = cv2.flip(mat,0,dst=None) #垂直镜像
    # cv2.imshow('hhh',mat1)
    # cv2.waitKey()
    im_hsv = cv2.cvtColor(mat1,cv2.COLOR_BGR2HSV) # 转换为HSV


    lower_red = np.array([0,100,100])
    upper_red = np.array([10,255,255])
    red_mask = cv2.inRange(im_hsv,lower_red,upper_red) # 红色区域取255（白色），其余取0（黑色）
    
    cv2.imshow('mask',red_mask)
    cv2.waitKey()

    binary = cv2.Canny(red_mask, 0, 60, apertureSize = 3)
    contours, cnt = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) # 提取矩形轮廓

    if len(contours) > 0:
        M = cv2.moments(contours[0])
        center_x = M["m10"] / M["m00"]
        center_y = M["m01"] / M["m00"]
        # ll = contours[0].mean(0)[0]
        # center_x = ll[0]
        # center_y = ll[1]
        res = ax2pos(zedPos[q],xAngle,center_x,center_y)
        tx.append(res[0])
        ty.append(res[1])
        r += 1

if r == 2:          
    targetX = (tx[0] + tx[1]) / 2
    targetY = (ty[0] + ty[1]) / 2
elif r == 1:
    targetX = tx[0]
    targetY = ty[0]
elif r == 0:
    print('error!')
    targetX = 0
    targetY = 0

print('r is ' + str(r))

_x = targetX
_y = targetY

print('_x is ' + str(_x))
print('_y is ' + str(_y))


print('restart!')

if pos[0] > _x:
    Xsign = -1
else: 
    Xsign = 1

if pos[1] > _y:
    Ysign = -1
else: 
    Ysign = 1


while abs(pos[0] - _x) > 0.05:
    pos[0] = pos[0] + Xsign * 0.01
    vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)


while abs(pos[1] - _y) > 0.05:
    pos[1] = pos[1] + Ysign * 0.01
    vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)


print('end!')



z = targetPos[2] + 0.1

if pos[2] > z:
    Zsign = -1
else: 
    Zsign = 1
print('land!')

start = time.time()
while time.time() - start < 0.1:
    vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

while abs(pos[2] - z) > 0.05:
    pos[2] = pos[2] + Zsign * 0.005
    vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

start = time.time()
while time.time() - start < 0.1:
    vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

print('scramb!')
start = time.time()
while time.time() - start < 0.1:
    vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
vrep.simxSetStringSignal(clientID,'goto_close','1',vrep.simx_opmode_oneshot) 


