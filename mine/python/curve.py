import numpy as np
import math
class CubicCurve:
    coeff = [0,0,0,0]
    start = [0,0]
    end = [0,0]
    start_derivative = 0.0
    end_derivative = 0.0

    def calcCoeff(self):
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

    def getVal(self, x):
        return self.coeff[0,0] * (x ** 3) + self.coeff[1,0] * (x ** 2) + self.coeff[2,0] * x + self.coeff[3,0]

    def getDerivative(self, x):
        return 3 * self.coeff[0,0] * (x ** 2) + 2 * self.coeff[1,0] * x + self.coeff[2,0]

    def getDerivative2(self,x):
        return 6 * self.coeff[0,0] * x + 2 * self.coeff[1,0]

    def sample(self, interval):
        sampled_data = []
        curr_x = self.start[0]
        while curr_x < self.end[0]:
            k = self.getDerivative(curr_x)
            curr_y = self.getVal(curr_x)
            sampled_data.append([curr_x,curr_y])
            curr_x += (interval / math.sqrt(k*k + 1))
        return sampled_data
