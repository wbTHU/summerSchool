import math
try:
    import vrep
except:
    print('import vrep failed')
    print('')

rad = 180 / math.pi
step = 0.018

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
        print("get Handle")

    # 打开起落架
    def bracketOpen(self):
        _, lOri = vrep.simxGetObjectOrientation(self.clientID, self.left, -1, vrep.simx_opmode_blocking)
        _, lPos = vrep.simxGetObjectPosition(self.clientID, self.left, -1, vrep.simx_opmode_blocking)
        print('lPos:', lPos)
        _, rOri = vrep.simxGetObjectOrientation(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        _, rPos = vrep.simxGetObjectPosition(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        print('rPos:', rPos)
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
        print('lPos:', lPos)
        _, rOri = vrep.simxGetObjectOrientation(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        _, rPos = vrep.simxGetObjectPosition(self.clientID, self.right, -1, vrep.simx_opmode_blocking)
        print('rPos:', rPos)
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
    def fly(self, des):
        if vrep.simxGetConnectionId(self.clientID) != -1:
            _, pos = vrep.simxGetObjectPosition(self.clientID, self.target, -1, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(self.clientID)
            vrep.simxGetPingTime(self.clientID)
            length = (des[0] - pos[0]) ** 2 + (des[1] - pos[1]) ** 2 + (des[2] - pos[2]) ** 2
            _round = int(length / step)
            dx = (des[0] - pos[0]) / _round
            dy = (des[1] - pos[1]) / _round
            dz = (des[2] - pos[2]) / _round
            for i in range(_round):
                pos[0] += dx
                pos[1] += dy
                pos[2] += dz
                print('pos:', pos)
                vrep.simxSetObjectPosition(self.clientID, self.target, -1, pos, vrep.simx_opmode_blocking)
                vrep.simxSynchronousTrigger(self.clientID)
                vrep.simxGetPingTime(self.clientID)
            vrep.simxSetObjectPosition(self.clientID, self.target, -1, des, vrep.simx_opmode_blocking)
            vrep.simxSynchronousTrigger(self.clientID)
            vrep.simxGetPingTime(self.clientID)

copter = Copter()
copter.bracketOpen()
copter.fly([1, 1, 2])
copter.fly([2, 3, 1])
copter.fly([0, 0, 1])
copter.bracketClose()