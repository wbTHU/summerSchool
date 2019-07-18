import numpy as np
import math

class CoordinateTransformer:

    trans_matrix = np.mat([[0, 0], [0, 0]])
    trans_offset = np.mat([[0], [0]])
    x_start_pos = [0,0]
    x_end_pos = [0,0]
    def __init__(self, x_start_pos, x_end_pos):
        self.x_start_pos = x_start_pos
        self.x_end_pos = x_end_pos
        co = np.dot(np.array(self.x_end_pos) - np.array(self.x_start_pos), np.array([1, 0])) / (
            np.linalg.norm(np.array(self.x_end_pos) - np.array(self.x_start_pos)))
        si = 0
        if self.x_end_pos[1] - self.x_start_pos[1] > 0:
            si = math.sqrt(1 - co ** 2)
        else:
            si = -math.sqrt(1 - co ** 2)
        self.trans_matrix = np.mat([[co, si], [-si, co]])
        self.trans_offset = np.mat([[-self.x_start_pos[0]], [-self.x_start_pos[1]]])

    def transPos(self, px, py):
        pv = np.mat([[px], [py]])
        new_pos = np.matmul(self.trans_matrix, pv + self.trans_offset)
        return new_pos[0, 0], new_pos[1, 0]

    def invTransPos(self, px, py):
        pv = np.mat([[px], [py]])
        print(pv)
        print(self.trans_matrix.I)
        new_pos = np.matmul(self.trans_matrix.I, pv) - self.trans_offset
        return new_pos[0, 0], new_pos[1, 0]

    def transDirection(self, px, py):
        nx_ori = self.transPos(0, 0)
        nx_end = self.transPos(px, py)
        return np.array(nx_end) - np.array(nx_ori)

    def invTransDirection(self, px, py):
        nx_ori = self.invTransPos(0.0)
        nx_end = self.invTransPos(px, py)
        return np.array(nx_end) - np.array(nx_ori)
