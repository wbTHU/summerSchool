import vrep
import numpy as np
import math
import time
import ctypes


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

_, originalP = vrep.simxGetObjectPosition(clientID,targetHandle,-1,vrep.simx_opmode_blocking)
_, landP = vrep.simxGetObjectPosition(clientID,landHandle,-1,vrep.simx_opmode_blocking)
pos = originalP

_, zedPos = vrep.simxGetObjectPosition(clientID,zedHandle[0],-1,vrep.simx_opmode_blocking)



vrep.simxSynchronousTrigger(clientID)

print('go out')

x = 1
y = -1
z = 3

if pos[0] > x:
    Xsign = -1
else: 
    Xsign = 1

if pos[1] > y:
    Ysign = -1
else: 
    Ysign = 1

if pos[2] > z:
    Zsign = -1
else: 
    Zsign = 1

while abs(pos[0] - x) > 0.1:
    pos[0] = pos[0] + Xsign * 0.005
    vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

while abs(pos[1] - y) > 0.1:
    pos[1] = pos[1] + Ysign * 0.005
    vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

while abs(pos[2] - z) > 0.1:
    pos[2] = pos[2] + Zsign * 0.005
    vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)



# vrep.simxGetPingTime(clientID)
    
# vision = []
# for i in range(zedNum):
#     _, sensor, v = vrep.simxGetVisionSensorImage(clientID,zedHandle[i],0,vrep.simx_opmode_blocking)
#     print(len(v))
#     f = open(zedName + str(i) +'.txt', 'w')
#     f.write(str(v))
#     f.close()
#     vision.append(v)
# vrep.simxPauseCommunication(clientID, True)
_, sensor, v = vrep.simxGetVisionSensorImage(clientID,zedHandle[0],0,vrep.simx_opmode_blocking)

print(len(v))
# print(max(v))
# f = open(zedName + str(0) +'.txt', 'w')
# f.write(str(v))
# f.close()
t = []
for i in v:
    t.append(int2uint8(i))
# print(max(t))
targetX = 0
targetY = 0


maxx=0
minx=100000
maxy=0
miny=100000
xlen = 1280
ylen = 720



for i in range(xlen): 
    for j in range(ylen):
        if v[i*ylen*3 + j*3]> 0 and v[i*ylen*3+j*3+1]> 0 and v[i*ylen*3+2]>0:
            maxx=max(maxx,i)
            minx=min(minx,i)
            maxy=max(maxy,j)
            miny=min(miny,j)
targetX = (maxx+minx)/2
targetY = (maxy+miny)/2
print(targetX)
print(targetY)
              




# _, buffer = vrep.simxGetStringSignal(clientID,'targetPos',vrep.simx_opmode_blocking)

# vrep.simxSynchronousTrigger(clientID)
# vrep.simxGetPingTime(clientID)
# err, buffer = vrep.simxGetStringSignal(clientID,'targetPos',vrep.simx_opmode_blocking)
# ss = buffer.decode('utf-8')
# s = ss.split(' ')

# targetX = round(float(s[0]))
# print(targetX)
# targetY = round(float(s[1]))

print('zed0Pos' + str(zedPos))
z = zedPos[2]
print('z is' + str(z))
d = math.tan(xAngle * math.pi / 360) * z / 640
_x = zedPos[0] + (targetX - 640) * d
_y = zedPos[1] + (targetY - 360) * d

print('d is' + str(d))
print('_x is'+ str(_x))
print('_y is'+ str(_y)) 



if vrep.simxGetConnectionId(clientID) != -1:

    # 运动
    # if abs(landP[2] - pos[2]) < 0.2:
    #     break
    # pos[2] = pos[2] - 0.01
    # vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_oneshot)

    
    # vrep.simxPauseCommunication(clientID, True)


    print('start to go back')


    if pos[0] > _x:
        Xsign = -1
    else: 
        Xsign = 1

    if pos[1] > _y:
        Ysign = -1
    else: 
        Ysign = 1

    while abs(pos[0] - _x) > 0.1:
        pos[0] = pos[0] + Xsign * 0.005
        vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)

    while abs(pos[1] - _y) > 0.1:
        pos[1] = pos[1] + Ysign * 0.005
        vrep.simxSetObjectPosition(clientID,targetHandle,-1,pos,vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)

    print('end?')
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
    
    
    





# imageBuffer = sim.getVisionSensorImage(zed_vision0)
#     maxx=0
#     minx=100000
#     maxy=0
#     miny=100000
#     xlen = 1280
#     ylen = 2160
#     for i=1,xlen,2 do  
#         for j=100,ylen-100,30 do
#             if (imageBuffer[i*ylen+j]>0.9 and imageBuffer[i*ylen+j+1]>0.9 and imageBuffer[i*ylen+j+2]>0.9) then
#                 maxx=math.max(maxx,i)
#                 minx=math.min(minx,i)
#                 maxy=math.max(maxy,j)
#                 miny=math.min(miny,j)
#             end
#         end
#     end  
#     targetx = (maxx + minx) / 2
#     targety = (maxy + miny) / 6
#     ss = tostring(targetx)..' '..tostring(targety)

#     sim.setStringSignal('targetPos',ss)