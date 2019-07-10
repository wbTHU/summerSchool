import math
try:
    import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import math
import time
rad = 180 / math.pi
print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP

vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
time.sleep(2)
#vrep.simxStartSimulation(clientID)
if clientID != -1:
    print('Connected to remote API server')
    vrep.simxSynchronous(clientID, True)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    _, lJoint = vrep.simxGetObjectHandle(clientID, "leftJoint", vrep.simx_opmode_blocking)
    _, rJoint = vrep.simxGetObjectHandle(clientID, "rightJoint", vrep.simx_opmode_blocking)
    # _, lSensor = vrep.simxGetObjectHandle(clientID, "left", vrep.simx_opmode_blocking)
    # _, rSensor = vrep.simxGetObjectHandle(clientID, "right", vrep.simx_opmode_blocking)
    #
    print("get Handle")
    # _, lJPos = vrep.simxGetJointPosition(clientID, "leftJoint", vrep.simx_opmode_blocking)
    # _, rJPos = vrep.simxGetJointPosition(clientID, "rightJoint", vrep.simx_opmode_blocking)
    start_time = time.time()
    while vrep.simxGetConnectionId(clientID) != -1:
        _, lJPos = vrep.simxGetJointPosition(clientID, lJoint, vrep.simx_opmode_buffer)
        # _, lJPos = vrep.simxGetJointPosition(clientID, lSensor, vrep.simx_opmode_buffer)
        print('lJPos:', lJPos)
        _, rJPos = vrep.simxGetJointPosition(clientID, rJoint, vrep.simx_opmode_buffer)
        # _, rJPos = vrep.simxGetJointPosition(clientID, rSensor, vrep.simx_opmode_buffer)
        t = time.time() - start_time
        print('target angle:', 180/rad * t * 100)
        vrep.simxSetJointTargetPosition(clientID, lJoint, 180/rad * t * 100, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID, rJoint, 180/rad * t * 100, vrep.simx_opmode_blocking)
        print(vrep.simxGetJointPosition(clientID, lJoint, vrep.simx_opmode_blocking))
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
