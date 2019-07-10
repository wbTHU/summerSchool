from numpy import array
import numpy as np
import math
from matplotlib.path import Path
from copy import deepcopy
import python.msg

epsilon = 1e-4


class ObjectBoundingBox:
    name = ''
    pos = [0.0,0.0,0.0]
    x = 0.0
    y = 0.0
    z = 0.0

    def __init__(self, name, pos, x, y, z):
        self.name = name
        self.pos = pos
        self.x = x
        self.y = y
        self.z = z
        return

    def getName(self):
        return self.name

    def collide(self, obb):
        if abs(self.pos[2] - obb.pos[2]) > self.z + obb.z + epsilon or math.sqrt((self.pos[0] - obb.pos[0]) ** 2 + (self.pos[1] - obb.pos[1])**2) > math.sqrt(self.x ** 2 + self.y ** 2) + math.sqrt(obb.x ** 2 + obb.y ** 2) + epsilon:
            return False;
        print('object.collide with')
        print(self.name)
        return True;

class GateCounter(ObjectBoundingBox):
    eulers = [0,0,0]
    type = 0
    x_n = [1,0,0]
    y_n = [0,1,0]
    z_n = [0,0,1]
    low_bound = []
    high_bound = []
    gate_in = [0,0,0]
    gate_out = []



    def __init__(self, name, pos, x, y, z, eulers, type):
        self.name = name
        self.pos = pos
        self.x = x
        self.y = y
        self.z = z
        self.eulers = eulers
        self.type = type

        R_x = np.array([[1,0,0],[0,math.cos(eulers[0]),math.sin(eulers[0])],[0,-math.sin(eulers[0]),math.cos(eulers[0])]])
        R_y = np.array([[math.cos(eulers[1]),0,math.sin(eulers[1])],[0,1,0],[-math.sin(eulers[1]),0,math.cos(eulers[1])]])
        R_z = np.array([[math.cos(eulers[2]),-math.sin(eulers[2]),0],[math.sin(eulers[2]),math.cos(eulers[2]),0],[0,0,1]])
        self.x_n = np.matmul(R_x, np.matmul(R_y, np.matmul(R_z,np.array(self.x_n)))).tolist()
        self.y_n = np.matmul(R_x, np.matmul(R_y, np.matmul(R_z,np.array(self.y_n)))).tolist()
        self.z_n = np.matmul(R_x, np.matmul(R_y, np.matmul(R_z,np.array(self.z_n)))).tolist()

        self.gate_in = (np.array(self.pos) + 0.5 * np.array(self.x_n)).tolist()
        self.gate_out = (np.array(self.pos) - 0.5 * np.array(self.x_n)).tolist()

        if type == 1:
            self.low_bound = [self.pos[0] + self.z * self.z_n[0] - 0.2, self.pos[1] + 0.5 * self.x_n[1],
                              self.pos[2] - self.y * self.y_n[2] - 0.2]
            self.high_bound = [self.pos[0] - self.z * self.z_n[0] + 0.2, self.pos[1] - 0.5 * self.x_n[1],
                               self.pos[2] + self.y * self.y_n[2] + 0.2]
        if type == 2:
            self.low_bound = [self.pos[0] - self.y * self.y_n[0] - 0.2, self.pos[1] + 0.5 * self.x_n[1],
                              self.pos[2] - self.z * self.z_n[2] - 0.2]
            self.high_bound = [self.pos[0] + self.y * self.y_n[0] + 0.2, self.pos[1] - 0.5 * self.x_n[1],
                               self.pos[2] + self.z * self.z_n[2] + 0.2]
        #print(self.x_n)
        #print(self.y_n)
        #print(self.z_n)
        return

    def collide(self, obb):

        if type == 1:
            edge1_x, edge1_y, edge1_z = self.pos + self.z * self.z_n
            edge2_x, edge2_y, edge2_z = self.pos - self.z * self.z_n
            dis_xy1 = math.sqrt((obb.pos[0] - edge1_x) ** 2 + (obb.pos[1] - edge1_y) ** 2)
            dis_xy2 = math.sqrt((obb.pos[0] - edge2_x) ** 2 + (obb.pos[1] - edge2_y) ** 2)
            obb_r = math.sqrt(obb.x ** 2 + obb.y **2)
            cross_d = abs(obb.pos[0] * self.z_n[1] - obb.pos[1] * self.z_n[0] -self.z_n[1] * self.pos[0] + self.z_n[0] * self.pos[1] ) / math.sqrt(self.z_n[1]**2 + self.z_n[0]**2)
            if obb.pos[2] - obb.z - epsilon > self.pos[2] + self.y or obb.pos[2] + obb.z + epsilon < self.pos[2] - self.y:
                return False
            if (obb.pos[2] < self.pos[2]+ self.y + obb.z +epsilon and obb.pos[2] > self.pos[2] + self.y - obb.z -epsilon) or (obb.pos[2] < self.pos[2]- self.y + obb.z +epsilon and obb.pos[2] > self.pos[2] - self.y - obb.z -epsilon):
                if obb_r + epsilon > cross_d and obb.pos[0] > self.pos[0] - self.z and obb.pos[0] < self.pos[0] + self.z:
                    print('true1')
                    return True
                return False
            if obb_r + epsilon > dis_xy1 or obb_r + epsilon > dis_xy2:
                print('true2')
                return True
            return False

        if type == 2:
            edge1_x, edge1_y, edge1_z = self.pos + self.y * self.y_n
            edge2_x, edge2_y, edge2_z = self.pos - self.y * self.y_n
            dis_xy1 = math.sqrt((obb.pos[0] - edge1_x) ** 2 + (obb.pos[1] - edge1_y) ** 2)
            dis_xy2 = math.sqrt((obb.pos[0] - edge2_x) ** 2 + (obb.pos[1] - edge2_y) ** 2)
            cross_d = abs(obb.pos[0] * self.y_n[1] - obb.pos[1] * self.y_n[0] -self.y_n[1] * self.pos[0] + self.y_n[0] * self.pos[1] ) / math.sqrt(self.y_n[1]**2 + self.y_n[0]**2)
            obb_r = math.sqrt(obb.x ** 2 + obb.y **2)
            if obb.pos[2] - obb.z - epsilon > self.pos[2] + self.z or obb.pos[2] + obb.z + epsilon < self.pos[2] - self.z:
                return False
            if (obb.pos[2] < self.pos[2] + self.z + obb.z + epsilon and obb.pos[2] > self.pos[2] + self.z - obb.z - epsilon) or (obb.pos[2] < self.pos[2] - self.z + obb.z + epsilon and obb.pos[2] > self.pos[2] - self.z - obb.z - epsilon):
                if obb_r + epsilon > cross_d and obb.pos[0] > self.pos[0] - self.y and obb.pos[0] < self.pos[0] + self.y:
                    return True
                return False
            if obb_r + epsilon > dis_xy1 or obb_r + epsilon > dis_xy2:
                return True
            return False
        return False


#gc = GateCounter([0,0,0],10,10,10,[0,-1.5707963705063,-1.5707963705063],'bala')





