import vrep
import numpy as np
import math
import time
import ctypes


tstep = 0.005
baseName = 'Quadricopter_base'
moveName = 'Quadricopter_target'
targetName = 'Target'
handName = 'JacoHand'
handSignal = 'close_hand'


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


    
_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)
_, moveHandle = vrep.simxGetObjectHandle(clientID, moveName, vrep.simx_opmode_blocking)
_, targetHandle = vrep.simxGetObjectHandle(clientID,targetName,vrep.simx_opmode_blocking)
err, scriptHandle = vrep.simxGetObjectHandle(clientID,'util_funcs',vrep.simx_opmode_blocking)

print('Handles available!')
   
# _, targetPos = vrep.simxGetObjectPosition(clientID,targetHandle,-1,vrep.simx_opmode_blocking)
vrep.simxSynchronousTrigger(clientID)
_, _, targetPos, _, _ = vrep.simxCallScriptFunction(clientID,'my_funcs',vrep.sim_scripttype_customizationscript,'my_get_target_platform_pos',[],[],[],'',vrep.simx_opmode_blocking)
_, _, endPos, _, _ = vrep.simxCallScriptFunction(clientID,'my_funcs',vrep.sim_scripttype_customizationscript,'my_get_end_point_pos',[],[],[],'',vrep.simx_opmode_blocking)

_, originalPos = vrep.simxGetObjectPosition(clientID,moveHandle,-1,vrep.simx_opmode_blocking)

print(str(targetPos))


# pos = originalPos


# x = targetPos[0]
# y = targetPos[1]
# z = 5


# if pos[0] > x:
#     Xsign = -1
# else: 
#     Xsign = 1

# if pos[1] > y:
#     Ysign = -1
# else: 
#     Ysign = 1

# if pos[2] > z:
#     Zsign = -1
# else: 
#     Zsign = 1

# print('start!')
# while abs(pos[2] - z) > 0.05:
#     pos[2] = pos[2] + Zsign * 0.01
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)
# start = time.time()
# while time.time() - start < 0.1:
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# while abs(pos[0] - x) > 0.05:
#     pos[0] = pos[0] + Xsign * 0.01
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# start = time.time()
# while time.time() - start < 0.1:
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# while abs(pos[1] - y) > 0.05:
#     pos[1] = pos[1] + Ysign * 0.01
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)
# start = time.time()
# while time.time() - start < 0.1:
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# print('end!')

# z = targetPos[2] + 0.1

# if pos[2] > z:
#     Zsign = -1
# else: 
#     Zsign = 1
# print('land!')

# start = time.time()
# while time.time() - start < 0.1:
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# while abs(pos[2] - z) > 0.05:
#     pos[2] = pos[2] + Zsign * 0.005
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# start = time.time()
# while time.time() - start < 0.1:
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)

# print('scramb!')
# start = time.time()
# while time.time() - start < 0.1:
#     vrep.simxSetObjectPosition(clientID,moveHandle,-1,pos,vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(clientID)
#     vrep.simxGetPingTime(clientID)
# vrep.simxSetStringSignal(clientID,'goto_close','1',vrep.simx_opmode_oneshot) 


