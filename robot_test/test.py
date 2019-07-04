import vrep
import numpy as np
import math
import time

RAD2DEG = 180 / math.pi
tstep = 0.005
jointNum = 6
baseName = 'Jaco'
jointName = 'Jaco_joint'


print ('program started!')
vrep.simxFinish(-1)

while True:
    clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
    if clientID != -1:
        break
    else:
        print ('Cannot connect to server!')
        time.sleep(0.2)

print ('Connection Success!')

vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot)
vrep.simxSynchronous(clientID,True)
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

jointHandle = np.zeros((jointNum,),dtype=np.int)
for i in range(jointNum):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i+1), vrep.simx_opmode_blocking)
    jointHandle[i] = returnHandle

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)

print('Handles available!')


jointConfig = np.zeros((jointNum,))
for i in range(jointNum):
     _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_streaming)
     jointConfig[i] = jpos

lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 記錄當前時間
vrep.simxSynchronousTrigger(clientID)  # 讓模擬走一步
# 開始模擬
while vrep.simxGetConnectionId(clientID) != -1:
    currCmdTime = vrep.simxGetLastCmdTime(clientID)  # 記錄當前時間
    dt = currCmdTime - lastCmdTime # 記錄時間間隔，用於控制

    lastCmdTime=currCmdTime    # 記錄當前時間
    vrep.simxSynchronousTrigger(clientID)  # 進行下一步
    vrep.simxGetPingTime(clientID)    # 使得該模擬步走完

    # 讀取當前的狀態值，之後都用buffer形式讀取
    for i in range(jointNum):
        _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_buffer)
        print(round(jpos * RAD2DEG, 2))
        jointConfig[i] = jpos

    # 控制命令需要同時方式，故暫停通訊，用於儲存所有控制命令一起傳送
    vrep.simxPauseCommunication(clientID, True)
    for i in range(jointNum):
        vrep.simxSetJointTargetPosition(clientID, jointHandle[i],  90/RAD2DEG, vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)


