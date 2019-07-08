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

import time

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')

    # # Now try to retrieve data in a blocking fashion (i.e. a service call):
    # res, objs = vrep.simxGetObjects(clientID, vrep.sim_handle_all, vrep.simx_opmode_blocking)
    # if res == vrep.simx_return_ok:
    #     print('Number of objects in the scene: ', len(objs))
    # else:
    #     print('Remote API function call returned with error code: ', res)
    #
    # time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    vrep.simxSynchronous(clientID, True)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    _, base = vrep.simxGetObjectHandle(clientID, "Quadricopter_base", vrep.simx_opmode_oneshot_wait)
    _, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_oneshot_wait)
    _, zed0 = vrep.simxGetObjectHandle(clientID, "zed_vision0", vrep.simx_opmode_oneshot_wait)
    print("get Handle")

    while vrep.simxGetConnectionId(clientID) != -1:
        _, pos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
        while abs(pos[2] - 1) > 0.1:
            _, pos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
            pos[2] += 0.005
            print(pos)
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
