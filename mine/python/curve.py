import numpy as np
import math
class CubicCurve: # 三次多项式曲线类
    coeff = [0,0,0,0] # 三次多项式的四个系数
    start = [0,0] # 左端点坐标
    end = [0,0] # 右端点坐标
    start_derivative = 0.0 #　左端点处一阶导
    end_derivative = 0.0 # 右端点处一阶导

    def calcCoeff(self): # 计算多项式系数
        print(type(self.start[1]))
        A = np.mat([[self.start[0]**3, self.start[0]**2, self.start[0], 1], [self.end[0]**3, self.end[0]**2, self.end[0], 1], [3*(self.start[0]**2), 2*self.start[0], 1, 0], [3*(self.end[0]**2), 2*self.end[0], 1, 0]])
        b = np.mat([[self.start[1]],[self.end[1]],[self.start_derivative],[self.end_derivative]])
        print(type(self.start[1]))
        print(type(self.end_derivative))
        self.coeff = np.matmul(A.I, b)

    def __init__(self, start, end, start_derivative, end_derivative):
        self.start = start
        self.end = end
        self.start_derivative = start_derivative
        self.end_derivative = end_derivative
        self.calcCoeff()

    def getVal(self, x): #　得到曲线上某一点的函数值
        return self.coeff[0,0] * (x ** 3) + self.coeff[1,0] * (x ** 2) + self.coeff[2,0] * x + self.coeff[3,0]

    def getDerivative(self, x): # 得到某一点的一阶导
        return 3 * self.coeff[0,0] * (x ** 2) + 2 * self.coeff[1,0] * x + self.coeff[2,0]

    def getDerivative2(self,x): # 得到某一点的二阶导
        return 6 * self.coeff[0,0] * x + 2 * self.coeff[1,0]

    def sample(self, interval): # 在曲线上等弧长取样, interval为取样间隔
        sampled_data = []
        curr_x = self.start[0]
        while curr_x < self.end[0]:
            k = self.getDerivative(curr_x)
            curr_y = self.getVal(curr_x)
            sampled_data.append([curr_x,curr_y])
            curr_x += (interval / math.sqrt(k*k + 1))
        return sampled_data
