# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
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

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res, objs = vrep.simxGetObjects(clientID, vrep.sim_handle_all, vrep.simx_opmode_blocking)
    if res == vrep.simx_return_ok:
        print('Number of objects in the scene: ', len(objs))
    else:
        print('Remote API function call returned with error code: ', res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    vrep.simxSynchronous(clientID, True)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    _, base = vrep.simxGetObjectHandle(clientID, "Quadricopter_base", vrep.simx_opmode_oneshot_wait)
    _, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_oneshot_wait)
    _, zed0 = vrep.simxGetObjectHandle(clientID, "zed_vision0", vrep.simx_opmode_oneshot_wait)
    print("get Handle")
    # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
    # print(error, transferInfo)
    # _, spos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
    while vrep.simxGetConnectionId(clientID) != -1:
        _, pos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
        while abs(1 - pos[0]) > 0.1:
            pos[0] += 0.01
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
            # vrep.simxSynchronousTrigger(clientID)
            _, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
            # _, transferInfo2 = vrep.simxGetStringSignal(clientID, "transferInfo2", vrep.simx_opmode_blocking)
            # _str = transferInfo.decode('utf-8').replace("\"", "").split(" ")

        while abs(-2 - pos[1]) > 0.1:
            pos[1] += 0.01
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
            _, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
            # _, transferInfo2 = vrep.simxGetStringSignal(clientID, "transferInfo2", vrep.simx_opmode_blocking)
            # _str = transferInfo.decode('utf-8').replace("\"", "").split(" ")

        while abs(2 - pos[2]) > 0.1:
            pos[2] += 0.01
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
            _, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
            # _, transferInfo2 = vrep.simxGetStringSignal(clientID, "transferInfo2", vrep.simx_opmode_blocking)
            # _str = transferInfo.decode('utf-8').replace("\"", "").split(" ")
        print("OOOOK")
        break

    # vrep.simxSynchronousTrigger(clientID)
    # vrep.simxGetPingTime(clientID)
    # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
    #
    vrep.simxSynchronousTrigger(clientID)
    error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
    _str = transferInfo.decode('utf-8').replace("\"", "").split(" ")

    # vrep.simxSynchronousTrigger(clientID)
    # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo2", vrep.simx_opmode_blocking)
    # _str2 = transferInfo.decode('utf-8').replace("\"", "").split(" ")

    _, vspos = vrep.simxGetObjectPosition(clientID, zed0, -1, vrep.simx_opmode_blocking)
    length = vspos[2] * math.tan(85 * math.pi / 360) / 640
    # des_x = vspos[0] - (640 - int((_str[0] + _str2[0])*0.5)) * length
    # des_y = vspos[1] - (360 - int(int(_str[1]+_str2[1])/6)) * length
    des_x = vspos[0] - (640 - int(_str[0])) * length
    des_y = vspos[1] - (360 - int(int(_str[1]) / 3)) * length
    flagx = math.copysign(1, des_x)
    flagy = math.copysign(1, des_y)
    print(vspos[2])
    print(length)
    print(vspos[0], vspos[1])
    print(des_x, des_y)
    # pos = [0, 0, 0]
    # pos_err = [0, 0, 0]
    # ori = [0, 0, 0]
    # ori_err = [0, 0, 0]

    # last = vrep.simxGetLastCmdTime(clientID)
    # vrep.simxSynchronousTrigger(clientID)
    # vrep.simxGetPingTime(clientID)
    # # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
    # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
    # print(error, transferInfo.decode('utf-8'))
    # vrep.simxSynchronousTrigger(clientID)
    # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
    # print(error, transferInfo.decode('utf-8'))

    while vrep.simxGetConnectionId(clientID) != -1:
        # time = time.time()
        # for i in range(3):
        # vrep.simxClearStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)

        _, pos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
        # vrep.simxSynchronousTrigger(clientID)
        # error, transferInfo = vrep.simxGetStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
        # _str = transferInfo.decode('utf-8').replace("\"", "").split(" ")

        # vrep.simxPauseCommunication(clientID, True) #single step
        # if pos[2] - spos[2] < 2:
        #     pos[2] += 0.01
        # else:
        #     pos[1] += 0.01
        # while abs(5 - pos[0]) > 0.1:
        #     pos[0] += 0.01
        #     vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
        #     vrep.simxSynchronousTrigger(clientID)
        #     vrep.simxGetPingTime(clientID)
        #
        # while abs(5 - pos[1]) > 0.1:
        #     pos[1] += 0.01
        #     vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
        #     vrep.simxSynchronousTrigger(clientID)
        #     vrep.simxGetPingTime(clientID)

        # pos[0] += 0.01
        # pos[1] += 0.02
        # print(pos)
        # _, vspos = vrep.simxGetObjectPosition(clientID, zed0, -1, vrep.simx_opmode_oneshot)
        # flagx = math.copysign(1, vspos[0])
        # flagy = math.copysign(1, vspos[1])
        while abs(pos[0] - des_x) > 0.1:
            pos[0] += 0.01 * flagx
            # des_x += 0.01*flagx
            print(pos)
            # if abs(pos[0]) < 0.1:
            #     break
            # vspos[0] -= 0.01 * flagx
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
        while abs(pos[1] - des_y) > 0.1:
            pos[1] += 0.01 * flagy
            # des_y += 0.01 * flagy
            print(pos)
            # if abs(pos[1]) < 0.1:
            #     break
            # vspos[1] -= 0.01 * flagy
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
        while pos[2] > 0.3:
            pos[2] -= 0.01
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
        # vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
        # vrep.simxPauseCommunication(clientID, False)
        # for j in range(3):
        #     vrep.simxSetObjectPosition(clientID, target, )

        # vrep.simxSynchronousTrigger(clientID)
        # vrep.simxGetPingTime(clientID)


