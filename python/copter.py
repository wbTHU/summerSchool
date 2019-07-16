import math
import numpy as np
try:
    import vrep
except:
    print('import vrep failed')
    print('')

rad = 180 / math.pi
# step = 0.018
destination = [4, 4, 2]

class Copter():
    def __init__(self, clientID = None):
        if clientID is None:
            print('program start')
            vrep.simxFinish(-1)  # 结束所有残余进程
            self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
        else:
            self.clientID = clientID
        vrep.simxSynchronous(self.clientID, True)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)
        _, self.left = vrep.simxGetObjectHandle(self.clientID, "left", vrep.simx_opmode_blocking)
        _, self.right = vrep.simxGetObjectHandle(self.clientID, "right", vrep.simx_opmode_blocking)
        _, self.base = vrep.simxGetObjectHandle(self.clientID, "Quadricopter_base", vrep.simx_opmode_oneshot_wait)
        _, self.target = vrep.simxGetObjectHandle(self.clientID, "Quadricopter_target", vrep.simx_opmode_oneshot_wait)
        # _, self.car = vrep.simxGetObjectHandle(self.clientID, "Car", vrep.simx_opmode_blocking)
        print("get Handle")

    # 打开起落架
    def bracketOpen(self):
        _, lOri = vrep.simxGetObjectOrientation(self.clientID, self.left, -1, vrep.simx_opmode_blocking)
        _, lPos = vrep.simxGetObjectPosition(self.clientID, self.left, -1, vrep.simx_opmode_blocking)
        print('open lPos:', lPos)
        _, rOri = vrep.simxGetObjectOrientation(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        _, rPos = vrep.simxGetObjectPosition(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        print('open rPos:', rPos)
        lOri[0] -= 1
        rOri[0] += 1
        lPos[2] += 0.18
        rPos[2] += 0.18
        lPos[1] -= 0.08
        rPos[1] += 0.08
        vrep.simxSetObjectOrientation(self.clientID, self.left, -1, lOri, vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(self.clientID, self.right, -1, rOri, vrep.simx_opmode_oneshot)
        vrep.simxSetObjectPosition(self.clientID, self.left, -1, lPos, vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.clientID, self.right, -1, rPos, vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(self.clientID)
        vrep.simxGetPingTime(self.clientID)

    # 合上起落架，准备降落
    def bracketClose(self):
        _, lOri = vrep.simxGetObjectOrientation(self.clientID, self.left, -1, vrep.simx_opmode_blocking)
        _, lPos = vrep.simxGetObjectPosition(self.clientID, self.left, -1, vrep.simx_opmode_blocking)
        print('close lPos:', lPos)
        _, rOri = vrep.simxGetObjectOrientation(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        _, rPos = vrep.simxGetObjectPosition(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        print('close rPos:', rPos)
        lOri[0] += 1
        rOri[0] -= 1
        lPos[2] -= 0.18
        rPos[2] -= 0.18
        lPos[1] += 0.08
        rPos[1] -= 0.08
        vrep.simxSetObjectOrientation(self.clientID, self.left, -1, lOri, vrep.simx_opmode_oneshot)
        vrep.simxSetObjectOrientation(self.clientID, self.right, -1, rOri, vrep.simx_opmode_oneshot)
        vrep.simxSetObjectPosition(self.clientID, self.left, -1, lPos, vrep.simx_opmode_blocking)
        vrep.simxSetObjectPosition(self.clientID, self.right, -1, rPos, vrep.simx_opmode_blocking)
        vrep.simxSynchronousTrigger(self.clientID)
        vrep.simxGetPingTime(self.clientID)

    # 向指定点飞行
    def fly(self, des, step = 0.018):
        if vrep.simxGetConnectionId(self.clientID) != -1:
            _, pos = vrep.simxGetObjectPosition(self.clientID, self.target, -1, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(self.clientID)
            vrep.simxGetPingTime(self.clientID)
            length = (des[0] - pos[0]) ** 2 + (des[1] - pos[1]) ** 2 + (des[2] - pos[2]) ** 2
            length = math.sqrt(length)
            _round = int(length / step)
            if _round != 0:
                # print('not zero')
                dx = (des[0] - pos[0]) / _round
                dy = (des[1] - pos[1]) / _round
                dz = (des[2] - pos[2]) / _round
            else:
                # print('zero')
                dx = des[0]
                dy = des[1]
                dz = des[2]
            # _, posCar = vrep.simxGetObjectPosition(self.clientID, self.car, -1, vrep.simx_opmode_blocking)
            for i in range(_round):
                pos[0] += dx
                pos[1] += dy
                pos[2] += dz
                # posCar[0] += carStep
                # print('pos:', pos)
                vrep.simxSetObjectPosition(self.clientID, self.target, -1, pos, vrep.simx_opmode_blocking)
                # vrep.simxSetObjectPosition(self.clientID, self.car, -1, posCar, vrep.simx_opmode_blocking)
                vrep.simxSynchronousTrigger(self.clientID)
                vrep.simxGetPingTime(self.clientID)

            vrep.simxSetObjectPosition(self.clientID, self.target, -1, des, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(self.clientID)
            vrep.simxGetPingTime(self.clientID)
            for i in range(3):
                vrep.simxSynchronousTrigger(self.clientID)
                vrep.simxGetPingTime(self.clientID)


    def land(self):
        _, car = vrep.simxGetObjectHandle(self.clientID, "Car", vrep.simx_opmode_blocking)
        _, desc = vrep.simxGetObjectPosition(self.clientID, car, -1, vrep.simx_opmode_blocking)
        # _, desc = vrep.simxGetObjectPosition(self.clientID, car, -1, vrep.simx_opmode_blocking)
        # rec = desc
        while vrep.simxGetConnectionId(self.clientID) != -1:
            _, desc = vrep.simxGetObjectPosition(self.clientID, car, -1, vrep.simx_opmode_blocking)
            _, posc = vrep.simxGetObjectPosition(self.clientID, self.target, -1, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(copter.clientID)
            vrep.simxGetPingTime(copter.clientID)
            if abs(desc[0] - posc[0]) < 0.1 and abs(desc[1] - posc[1]) < 0.1:
                for i in range(8):
                    vrep.simxSynchronousTrigger(copter.clientID)
                    vrep.simxGetPingTime(copter.clientID)
                print("right on the car, ready to land")
                self.bracketClose()
                desc[2] += 0.1
                self.fly(desc, step=0.008)
                break
            desc[2] = destination[2]
            print('des:', desc)
            self.fly(desc)

copter = Copter()
copter.bracketOpen()
copter.fly(destination)
copter.land()
# copter.bracketClose()
# point = [0, 0, 0]
# _, car = vrep.simxGetObjectHandle(copter.clientID, "Car", vrep.simx_opmode_blocking)
# err, des = vrep.simxGetObjectPosition(copter.clientID, car, -1, vrep.simx_opmode_blocking)
# while vrep.simxGetConnectionId(copter.clientID) != -1:
#     # for i in range():
#     err, des = vrep.simxGetObjectPosition(copter.clientID, car, -1, vrep.simx_opmode_blocking)
#     vrep.simxSynchronousTrigger(copter.clientID)
#     vrep.simxGetPingTime(copter.clientID)
#     # if abs(des[1] - point[1]) > 0.1:
#     #     des[0] = des[0]
#     #     des[1] = des[1]
#     # else:
#     #     des[0] = des[0] + 0.005
#     des[2] = destination[2]
#     print('des:', des)
#     # point = des
#     copter.fly(des)
# copter.bracketClose()
