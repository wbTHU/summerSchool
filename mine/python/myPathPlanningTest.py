import numpy as np
from objectBoundingBox import *
from coordinateTransform import *
from curve import *
import copy
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
import sys

objects = []

gates = []


object_names = ['Tree','Tree#0','Cylinder','End','Target_platform','UR3','UR3#0']

gate_names = ['GateCounter_55cmX40cm', 'GateCounter_55cmX40cm#0', 'GateCounter_55cmX40cm#1','GateCounter_80cmX190cm', 'GateCounter_80cmX190cm#0', 'GateCounter_80cmX190cm#1', 'GateCounter_80cmX190cm#2'  ]



def dist(pos_x, pos_y):

    return np.linalg.norm(np.array(pos_x) - np.array(pos_y))

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


def getObjectsInformation(clientID):
    for i in range(0, len(object_names)):
        object_name = object_names[i]
        print(object_name)
        _, object_handle = vrep.simxGetObjectHandle(clientID, object_name, vrep.simx_opmode_blocking)
        _, object_pos = vrep.simxGetObjectPosition(clientID, object_handle, -1, vrep.simx_opmode_blocking)
        _, object_x = vrep.simxGetObjectFloatParameter(clientID, object_handle, 18, vrep.simx_opmode_blocking)
        _, object_y = vrep.simxGetObjectFloatParameter(clientID, object_handle, 19, vrep.simx_opmode_blocking)
        _, object_z = vrep.simxGetObjectFloatParameter(clientID, object_handle, 20, vrep.simx_opmode_blocking)
        if (object_name[0:3] == 'UR3'):
            object_x = 0.25
            object_y = 0.25
            object_z = 1

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
        if (i < 3):
            type = 1
        else:
            type = 2
        gates.append(GateCounter(object_name, object_pos, object_x, object_y, object_z, eulers, type))





def collideWithObstacles(quadcopter, obstacles):
    for obstacle in obstacles:
        if obstacle.collide(quadcopter):
            return True
    return False

def isStateVaild(state):
    quadcopter = ObjectBoundingBox('quad', state, 0.25,0.25,0.25)
    if collideWithObstacles(quadcopter, objects):
        return False
    if collideWithObstacles(quadcopter, gates):
        return False
    return state[0] >= -15 and state[0] <= 15 and state[1] >= -15 and state[1] <= 15 and state[2] >= -15 and state[3] <= 15

def canFlyDirectTo(start, goal):
    flag = True
    curr = copy.copy(start)
    step = (goal - start) / np.linalg.norm(goal - start) * 0.01
    while np.linalg.norm(goal - curr) > 0.006:
        curr = curr + step
        if (isStateVaild(curr) == False):
            return False
    return True



def flyDirectlyTo(clientID,  goal):
    _, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_oneshot_wait)
    _, base = vrep.simxGetObjectHandle(clientID, "Quadricopter_base", vrep.simx_opmode_oneshot_wait)
    _, pos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
    route = np.array([goal[0] - pos[0], goal[1] - pos[1], goal[2] - pos[2]])
    routeLen = np.linalg.norm(route)
    step = route / routeLen * 0.01
    speed = 2
    while np.linalg.norm(np.array(pos) - np.array(goal)) > speed * 0.011 / 2:
        pos[0] += speed * step[0]
        pos[1] += speed * step[1]
        pos[2] += speed * step[2]

        vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(clientID)
        vrep.simxGetPingTime(clientID)
    return



def flyTo(clientID, start, goal, start_derivative_v, end_derivative_v):

    _, target = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_oneshot_wait)
    _, base = vrep.simxGetObjectHandle(clientID, "Quadricopter_base", vrep.simx_opmode_oneshot_wait)

    coor_transformer = CoordinateTransformer(objects[4].pos[0:2], objects[3].pos[0:2])
    nx_start = coor_transformer.transPos(start[0], start[1])
    nx_goal = coor_transformer.transPos(goal[0], goal[1])
    nx_start_derivative_v = coor_transformer.transDirection(start_derivative_v[0], start_derivative_v[1])
    nx_end_derivative_v = coor_transformer.transDirection(end_derivative_v[0], end_derivative_v[1])
    nx_start_derivative = nx_start_derivative_v[1] / nx_start_derivative_v[0]
    nx_end_derivative = nx_end_derivative_v[1] / nx_end_derivative_v[0]
    cubic_curve = CubicCurve(nx_start, nx_goal, nx_start_derivative, nx_end_derivative)
    sampled_data = cubic_curve.sample(0.1)
    speed = 0

    for i in range(0, len(sampled_data)):
        x, y = coor_transformer.invTransPos(sampled_data[i][0], sampled_data[i][1])
        des = [x, y, 0.5]
        if abs(cubic_curve.getDerivative2(sampled_data[i][0])) < 0.6:
            speed = 2.0
        else:
            speed = 0.7

        _, pos = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
        route = np.array(des) - np.array(pos)
        routeLen = np.linalg.norm(route)
        step = route / routeLen * 0.01

        while np.linalg.norm(np.array(pos) - np.array(des)) > speed * 0.011 / 2:
            pos[0] += speed * step[0]
            pos[1] += speed * step[1]
            pos[2] += speed * step[2]
            vrep.simxSetObjectPosition(clientID, target, -1, pos, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(clientID)
            vrep.simxGetPingTime(clientID)
        i += 1
    return

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

    getObjectsInformation(clientID)

    target_platform_pos = objects[4].pos[0], objects[4].pos[1], 0.5
    end_pos = objects[3].pos[0], objects[3].pos[1], 1.8
    choose_gates = globalPathPlanning()

    flyTo(clientID, target_platform_pos, gates[choose_gates[0]].pos, [-1,0], [-gates[choose_gates[0]].x_n[0], -gates[choose_gates[0]].x_n[1]])
    flyTo(clientID, gates[choose_gates[0]].pos, gates[choose_gates[1]].pos, [-gates[choose_gates[0]].x_n[0], -gates[choose_gates[0]].x_n[1]], [-gates[choose_gates[1]].x_n[0], -gates[choose_gates[1]].x_n[1]])
    flyTo(clientID, gates[choose_gates[1]].pos, [-1, -3],[-gates[choose_gates[1]].x_n[0], -gates[choose_gates[1]].x_n[1]],[-1,1])
    flyTo(clientID, [-1, -3], gates[choose_gates[2]].pos, [-1, 1], [-gates[choose_gates[2]].x_n[0], -gates[choose_gates[2]].x_n[1]])
    flyDirectlyTo(clientID, end_pos)

