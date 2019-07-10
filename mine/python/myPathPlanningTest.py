import numpy as np
from ompl import base as ob
from ompl import geometric as og
from object_bounding_box import *


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
import sys

objects = []

gates = []


object_names = ['Tree','Tree#0','Cylinder','End','Target_platform','UR3','UR3#1']

gate_names = ['GateCounter_55cmX40cm', 'GateCounter_55cmX40cm#0', 'GateCounter_55cmX40cm#1','GateCounter_80cmX190cm', 'GateCounter_80cmX190cm#0', 'GateCounter_80cmX190cm#1', 'GateCounter_80cmX190cm#2'  ]

def dist(pos_x, pos_y):

    return np.linalg.norm(np.array(pos_x) - np.array(pos_y))

def getBounds(pos_x, pos_y):
    low_bound = [min(pos_x[0], pos_y[0]) - 1, min(pos_x[1], pos_y[1]) - 1, min(pos_x[2], pos_y[2]) - 1]
    high_bound = [max(pos_x[0], pos_y[0]) + 1, max(pos_x[1], pos_y[1]) + 1, max(pos_x[2], pos_y[2]) + 1]

    return low_bound, high_bound

def globalPathPlanning():
    choose_gate = [-1,-1,-1]
    min_distance = sys.maxsize
    target_platform_pos = objects[4].pos
    end_pos = objects[3].pos
    for i in range(0, len(gates)):
        for j in range(0, len(gates)):
            if i == j:
                continue
            for k in range(0, len(gates)):
                if k == j or k == i:
                    continue
                distance = dist(target_platform_pos, gates[i].pos) + dist(gates[i].pos, gates[j].pos) + dist(gates[j].pos, gates[k].pos) + dist(gates[k].pos, end_pos)
                if distance < min_distance:
                    min_distance = distance
                    chooose_gate = [i, j, k]

    return chooose_gate




def collideWithObstacles(quadcopter, obstacles):
    for obstacle in obstacles:
        if obstacle.collide(quadcopter):
            return True
    return False


def crossGate(clientID, gate, strategy):


    flyTo(clientID, gate.gate_in, gate.gate_out, gate.low_bound, gate.high_bound, strategy)



def flyTo(clientID, _start, _goal, bounds_low, bounds_high, strategy):
    def isStateVaild(state):
        quadcopter = ObjectBoundingBox('quad', [state.getX(),state.getY(),state.getZ()], 0.25,0.25,0.25)
        if collideWithObstacles(quadcopter, objects):
            return False
        if collideWithObstacles(quadcopter, gates):
            return False
        return state.getX() >= bounds_low[0] and state.getX() <= bounds_high[0] and state.getY() >= bounds_low[1] and state.getY() <= bounds_high[1] and state.getZ() >= bounds_low[2] and state.getZ() <= bounds_high[2]

    space = ob.SE3StateSpace()

    bounds = ob.RealVectorBounds(3)
    bounds.setLow(0, bounds_low[0])
    bounds.setLow(1, bounds_low[1])
    bounds.setLow(2, bounds_low[2])

    bounds.setHigh(0, bounds_high[0])
    bounds.setHigh(1, bounds_high[1])
    bounds.setHigh(2, bounds_high[2])
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)

    start = ob.State(space)
    start().setX(_start[0])
    start().setY(_start[1])
    start().setZ(_start[2])
    start().rotation().setIdentity()

    goal = ob.State(space)
    goal().setX(_goal[0])
    goal().setY(_goal[1])
    goal().setZ(_goal[2])
    goal().rotation().setIdentity()

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateVaild))
    ss.setStartAndGoalStates(start, goal, 0.05)

    planner = strategy(ss.getSpaceInformation())
    planner.setRange(0.01)
    # print(planner.getGoalBias())
    ss.setPlanner(planner)
    ss.setup()

    solved = ss.solve(20)
    # print the path to screen
    print("Found solution:\n%s" % ss.getSolutionPath())
    retPath = []
    solutionPath = ss.getSolutionPath()

    _, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_oneshot_wait)

    i = 0

    while i < solutionPath.getStateCount() and vrep.simxGetConnectionId(clientID) != -1:
        # time = time.time()
        # for i in range(3):
        # vrep.simxClearStringSignal(clientID, "transferInfo", vrep.simx_opmode_blocking)
        print("sadas")

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

        des_x = solutionPath.getState(i).getX()

        des_y = solutionPath.getState(i).getY()

        des_z = solutionPath.getState(i).getZ()



        des = [des_x, des_y, des_z]
        route = np.array([des_x - pos[0], des_y - pos[1], des_z - pos[2]])
        routeLen = np.linalg.norm(route)
        step = route / routeLen * 0.01


        while(np.linalg.norm(np.array(pos) - np.array(des)) > 0.1):
            pos[0] += step[0]
            pos[1] += step[1]
            pos[2] += step[2]
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
        i += 1


'''
def plan():
    space = ob.SE3StateSpace()

    bounds = ob.RealVectorBounds(3)
    bounds.setLow(0, -15)
    bounds.setLow(1, -15)
    bounds.setLow(2, 0)

    bounds.setHigh(0, 15)
    bounds.setHigh(1, 15)
    bounds.setHigh(2, 5)
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)

    start = ob.State(space)
    start().setX(10.6750)
    start().setY(0.4250)
    start().setZ(1)
    start().rotation().setIdentity()

    goal = ob.State(space)
    goal().setX(7.2)
    goal().setY(-10.4)
    goal().setZ(1)
    goal().rotation().setIdentity()

    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    ss.setStartAndGoalStates(start, goal, 0.05)

    planner = og.RRT(ss.getSpaceInformation())
    #print(planner.getGoalBias())
    ss.setPlanner(planner)
    ss.setup()

    solved = ss.solve(20)
    # print the path to screen
    print("Found solution:\n%s" % ss.getSolutionPath())
    retPath = []
    solutionPath = ss.getSolutionPath()
    for i in range(0, solutionPath.getStateCount()):
        print("X = %f\n" % solutionPath.getState(i).getX())
        print("Y = %f\n" % solutionPath.getState(i).getY())
        print("Z = %f\n" % solutionPath.getState(i).getZ())
        retPath.append([solutionPath.getState(i).getX(),solutionPath.getState(i).getY(),solutionPath.getState(i).getZ()])
    return retPath
    
    
'''




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

    vrep.simxSynchronous(clientID, True)
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    for i in range(0, len(object_names)):
        object_name = object_names[i]
        print(object_name)
        _, object_handle = vrep.simxGetObjectHandle(clientID, object_name, vrep.simx_opmode_blocking)
        _, object_pos = vrep.simxGetObjectPosition(clientID, object_handle, -1, vrep.simx_opmode_blocking)
        _, object_x = vrep.simxGetObjectFloatParameter(clientID, object_handle, 18, vrep.simx_opmode_blocking)
        _, object_y = vrep.simxGetObjectFloatParameter(clientID, object_handle, 19, vrep.simx_opmode_blocking)
        _, object_z = vrep.simxGetObjectFloatParameter(clientID, object_handle, 20, vrep.simx_opmode_blocking)
        objects
        print(object_pos)
        print(object_x)
        print(object_y)
        print(object_z)

        objects.append(ObjectBoundingBox(object_name, object_pos, object_x, object_y, object_z))

    for i in range(0, len(gate_names)):
        object_name = gate_names[i]
        print(object_name)
        _, object_handle = vrep.simxGetObjectHandle(clientID, object_name, vrep.simx_opmode_blocking)
        _, object_pos = vrep.simxGetObjectPosition(clientID, object_handle, -1, vrep.simx_opmode_blocking)
        _, object_x = vrep.simxGetObjectFloatParameter(clientID, object_handle, 18, vrep.simx_opmode_blocking)
        _, object_y = vrep.simxGetObjectFloatParameter(clientID, object_handle, 19, vrep.simx_opmode_blocking)
        _, object_z = vrep.simxGetObjectFloatParameter(clientID, object_handle, 20, vrep.simx_opmode_blocking)
        _, eulers = vrep.simxGetObjectOrientation(clientID, object_handle, -1, vrep.simx_opmode_blocking)
        print(object_pos)
        print(object_x)
        print(object_y)
        print(object_z)
        print(eulers)
        type = 0
        if(i < 3):
            type = 1
        else:
            type = 2
        gates.append(GateCounter(object_name, object_pos, object_x, object_y, object_z, eulers, type))

    choose_gates = globalPathPlanning()

    target_platform_pos = [7.2, -10.4, 1.0]

    end_pos = [-7.45, 8.2, 1.0]





    flyTo(clientID, target_platform_pos, gates[choose_gates[0]].gate_in, getBounds(target_platform_pos, gates[choose_gates[0]].gate_in)[0], getBounds(target_platform_pos, gates[choose_gates[0]].gate_in)[1], og.RRT)
    crossGate(clientID, gates[choose_gates[0]], og.RRT)
    flyTo(clientID, gates[choose_gates[0]].gate_out, gates[choose_gates[1]].gate_in, getBounds(gates[choose_gates[0]].gate_out, gates[choose_gates[1]].gate_in)[0], getBounds(gates[choose_gates[0]].gate_out, gates[choose_gates[1]].gate_in)[1], og.RRT)
    crossGate(clientID, gates[choose_gates[1]], og.RRT)
    flyTo(clientID, gates[choose_gates[1]].gate_out, gates[choose_gates[2]].gate_in,
          getBounds(gates[choose_gates[1]].gate_out, gates[choose_gates[2]].gate_in)[0],
          getBounds(gates[choose_gates[1]].gate_out, gates[choose_gates[2]].gate_in)[1], og.RRT)
    crossGate(clientID, gates[choose_gates[2]], og.RRT)
    flyTo(clientID, gates[choose_gates[2]].gate_out, end_pos,
          getBounds(gates[choose_gates[2]].gate_out, end_pos)[0],
          getBounds(gates[choose_gates[2]].gate_out, end_pos)[1], og.RRT)

    