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
        s[0], s[2] = s[2], s[0]  # opencv使用的是BGR，因此需要交换G和B
        t.append(s)
    re.append(t)


mat = np.array(re,dtype=np.uint8)
# mat1 = mat
mat2 = cv2.flip(mat,0,dst=None) #垂直镜像
mat1 = cv2.blur(mat2,(5,5)) # 模糊化降噪处理

# cv2.imshow('hhh',mat1)
# cv2.waitKey()


# -- 二维码识别 --

# gray = cv2.cvtColor(mat1,cv2.COLOR_BGR2GRAY) #进行灰度转化
# # cv2.imshow('hhh',gray)
# # cv2.waitKey()

# limit = 50 
# mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0），（存疑）达到凸显二维码的目的


# # 先找到白色大块边界（去除边界黑块影响）
# # 利用黑色像素的位置得到二维码的中心坐标

# MAXX=0
# MINX=100000
# MAXY=0
# MINY=100000
# xlen = 1280
# ylen = 720

# for i in range(xlen): 
#     for j in range(ylen):
#         if mask[j][i] > 0: # 找白色
#             MAXX=max(MAXX,i)
#             MINX=min(MINX,i)
#             MAXY=max(MAXY,j)
#             MINY=min(MINY,j)
#             # 得到白色边界
# print(MINX)
# print(MAXX)
# print(MINY)
# print(MAXY)


# maxx=0
# minx=100000
# maxy=0
# miny=100000
# xlen = 1280
# ylen = 720

# for i in range(xlen): 
#     for j in range(ylen):
#         if mask[j][i] == 0 and i >= MINX and i <= MAXX and j >= MINY and j <= MAXY: # 找黑色
#             maxx=max(maxx,i)
#             minx=min(minx,i)
#             maxy=max(maxy,j)
#             miny=min(miny,j)


# targetX = (maxx + minx) / 2
# targetY = (maxy + miny) / 2
# print(targetX)
# print(targetY)

# cv2.imshow('mask',mask)
# cv2.waitKey()

# -- 二维码识别 --


# -- 圆柱体物体识别 --

# im_hsv = cv2.cvtColor(mat1,cv2.COLOR_BGR2HSV) # 转换为HSV


# lower_red = np.array([0,100,100])
# upper_red = np.array([10,255,255])
# red_mask = cv2.inRange(im_hsv,lower_red,upper_red) # 红色区域取255（白色），其余取0（黑色）

# binary = cv2.Canny(red_mask, 0, 60, apertureSize = 3)
# contours, cnt = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) # 提取矩形轮廓

# if len(contours) > 0:
#     ll = contours[0].mean(0)[0]

#     center_x = ll[0]
#     center_y = ll[1]
#     print(center_x)
#     print(center_y) # 找到的像素点坐标（也可选择不取整）
#     # red_mask[center_y][center_x] = 0
#     cv2.imshow('mask',red_mask)
#     cv2.waitKey()


# -- 圆柱体物体识别 --

# -- T、E台子位置识别 --（类似二维码位置检查）(增加模糊化预处理后准确度明显上升  二维码也可采用)

gray = cv2.cvtColor(mat1,cv2.COLOR_BGR2GRAY) #进行灰度转化
# cv2.imshow('hhh',gray)
# cv2.waitKey()

limit = 50 
mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0），（存疑）达到凸显台子图案的目的


# 先找到白色大块边界（去除边界黑块影响）
# 利用黑色像素的位置得到中心坐标

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
print(MINX)
print(MAXX)
print(MINY)
print(MAXY)


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

kk = mat2[miny:maxy, minx:maxx]

cv2.imshow('hhhhhh',kk)
cv2.waitKey()


targetX = (maxx + minx) / 2
targetY = (maxy + miny) / 2
print(targetX)
print(targetY)




# -- T、E台子位置识别 --


# -- 寻找二维码新方法  -- 

# limit = 50
# tx = []
# ty = []
# masks = []

# row = 720
# col = 1280


# tx = []
# ty = []

# for q in range(zedNum):
#     v = trans[q]
#     re = []
#     for i in range(row):
#         t = []
#         for j in range(col):
#             s = []
#             for k in range(3):
#                 m = v[i * col * 3 + j * 3 + k]
#                 s.append(m)
#             s[0], s[2] = s[2], s[0]  # opencv使用的是BGR，因此需要交换G和B
#             t.append(s)
#         re.append(t)
    
#     mat = np.array(re,dtype=np.uint8)
#     mat1 = cv2.flip(mat,0,dst=None) #垂直镜像
#     gray = cv2.cvtColor(mat1,cv2.COLOR_BGR2GRAY)
#     mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0）,达到凸显二维码的目的
#     masks.append(mask)

#     kernel = np.ones((5,5),np.uint8)
#     erosion = cv2.erode(mask,kernel)
#     erosion = cv2.erode(erosion,kernel)

#     contours,hier=cv2.findContours(erosion, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
#     rec = []
#     for c in contours:
#         x,y,w,h=cv2.boundingRect(c)#计算出一个简单地边界框
#         if (abs(w-h)<10) & (w>10): # 判断得到的矩形的大小是否符合（二维码的某个块即可）
#             rec.append([x,y,w,h])

#     mi = -1
#     mw = 0
#     l = len(rec)
#     print(l)
#     for i in range(l):
#         x,y,w,h = rec[i]
#         if (mw < w):
#             mw = w
#             mi = i
#     if mi > -1:
#         x,y,w,h = rec[mi]
#         # mmm = mat1[y:y+h, x:x+w]
#         # cv2.imshow('mmm',mmm)
#         # cv2.waitKey()
#         targetX = x + w / 2
#         targetY = y + h / 2
#         tx.append(targetX)
#         ty.append(targetY)
#         print(targetX)
#         print(targetY)
#     else:
#         print(str(q) + ' is error!')


# -- 寻找二维码新方法  -- 





