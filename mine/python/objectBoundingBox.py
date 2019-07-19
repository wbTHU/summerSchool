from numpy import array
import numpy as np
import math
from matplotlib.path import Path
from copy import deepcopy
import python.msg

# 碰撞判断偏差
epsilon = 1e-4


class ObjectBoundingBox:
    name = ''
    pos = [0.0,0.0,0.0]     #包围盒中心坐标
    #包围盒的半长,半宽,半高
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
            return False
        return True

class GateCounter(ObjectBoundingBox):
    eulers = [0,0,0]
    # 门的种类, 1代表55cmX40cm, 2代表80cmX190cm
    type = 0
    # 在门坐标系下的x,y,z方向向量
    x_n = [1,0,0]
    y_n = [0,1,0]
    z_n = [0,0,1]

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

        return

    def collide(self, obb):

        if self.type == 1:
            edge1_x, edge1_y, edge1_z = np.array(self.pos) + self.z * np.array(self.z_n)
            edge2_x, edge2_y, edge2_z = np.array(self.pos) - self.z * np.array(self.z_n)
            dis_xy1 = math.sqrt((obb.pos[0] - edge1_x) ** 2 + (obb.pos[1] - edge1_y) ** 2)
            dis_xy2 = math.sqrt((obb.pos[0] - edge2_x) ** 2 + (obb.pos[1] - edge2_y) ** 2)
            obb_r = math.sqrt(obb.x ** 2 + obb.y **2)
            cross_d = abs(obb.pos[0] * self.z_n[1] - obb.pos[1] * self.z_n[0] -self.z_n[1] * self.pos[0] + self.z_n[0] * self.pos[1] ) / math.sqrt(self.z_n[1]**2 + self.z_n[0]**2)
            # z方向上距离过大, 不可能碰撞
            if obb.pos[2] - obb.z - epsilon > self.pos[2] + self.y or obb.pos[2] + obb.z + epsilon < self.pos[2] - self.y:
                return False
            # 与上方横梁碰撞
            if (obb.pos[2] < self.pos[2]+ self.y + obb.z +epsilon and obb.pos[2] > self.pos[2] + self.y - obb.z -epsilon) or (obb.pos[2] < self.pos[2]- self.y + obb.z +epsilon and obb.pos[2] > self.pos[2] - self.y - obb.z -epsilon):
                if obb_r + epsilon > cross_d and obb.pos[0] > self.pos[0] - self.z and obb.pos[0] < self.pos[0] + self.z:
                    return True
                return False
            # 与立柱碰撞
            if obb_r + epsilon > dis_xy1 or obb_r + epsilon > dis_xy2:
                return True
            return False

        if self.type == 2:
            edge1_x, edge1_y, edge1_z = np.array(self.pos) + self.y * np.array(self.y_n)
            edge2_x, edge2_y, edge2_z = np.array(self.pos) - self.y * np.array(self.y_n)
            dis_xy1 = math.sqrt((obb.pos[0] - edge1_x) ** 2 + (obb.pos[1] - edge1_y) ** 2)
            dis_xy2 = math.sqrt((obb.pos[0] - edge2_x) ** 2 + (obb.pos[1] - edge2_y) ** 2)
            cross_d = abs(obb.pos[0] * self.y_n[1] - obb.pos[1] * self.y_n[0] -self.y_n[1] * self.pos[0] + self.y_n[0] * self.pos[1] ) / math.sqrt(self.y_n[1]**2 + self.y_n[0]**2)
            obb_r = math.sqrt(obb.x ** 2 + obb.y **2)
            # z方向上距离过大, 不可能碰撞
            if obb.pos[2] - obb.z - epsilon > self.pos[2] + self.z or obb.pos[2] + obb.z + epsilon < self.pos[2] - self.z:
                return False
            # 与上方横梁碰撞
            if (obb.pos[2] < self.pos[2] + self.z + obb.z + epsilon and obb.pos[2] > self.pos[2] + self.z - obb.z - epsilon) or (obb.pos[2] < self.pos[2] - self.z + obb.z + epsilon and obb.pos[2] > self.pos[2] - self.z - obb.z - epsilon):
                if obb_r + epsilon > cross_d and obb.pos[0] > self.pos[0] - self.y and obb.pos[0] < self.pos[0] + self.y:
                    return True
                return False
            # 与立柱碰撞
            if obb_r + epsilon > dis_xy1 or obb_r + epsilon > dis_xy2:
                return True
            return False
        return False





