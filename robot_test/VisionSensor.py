import vrep
import numpy as np
import math
import cv2

def int2uint8(num):
    if num < 0:
        return num + 2 ** 8
    else:
        return num


class VisionSensor():

    def __init__(self, clientID, zedName):
        self.clientID = clientID
        self.zedName = zedName
        _, self.zedHandle = vrep.simxGetObjectHandle(clientID,zedName,vrep.simx_opmode_blocking)
        self.getPos()
        self.getImage()
        self.getDepthMap()

    def ax2pos(self,xangle, x, y, ori): # 图像坐标系向全局坐标系的转化（不是很准确） ori为整体的orientation，考虑转向
        z = abs(self.pos[2])
        d = math.tan(xangle * math.pi / 360) * z / 640

        theta = ori[2] # 转向角(弧度制) 

        _x = self.pos[0] - ((y - 360) * d * math.cos(theta) - (x - 640) * d * math.sin(theta))
        _y = self.pos[1] - ((x - 640) * d * math.cos(theta) + (y - 360) * d * math.sin(theta))

        Dm = 5e-3
        Df = 5e3

        dep = Dm +   (Df - Dm )* self.depthMap[int(y)][int(x)]
        _z = self.pos[2] - dep
        res = []
        res.append(_x)
        res.append(_y)
        res.append(_z)
        return res

    
    def getDepthMap(self):
        _, sensor, self.depthMap = vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.zedHandle,vrep.simx_opmode_blocking)
        self.depthMap = np.array(self.depthMap).reshape(720,1280)
        # self.sensor = sensor

    def getDepth(self, x, y):
        self.getDepthMap()
        return self.depthMap[x + y * 1280]
        

    def getPos(self):
        _, self.pos = vrep.simxGetObjectPosition(self.clientID,self.zedHandle,-1,vrep.simx_opmode_blocking)
    
    def getImage(self):
        if self.zedHandle != None:
            vrep.simxSynchronousTrigger(self.clientID)
            _, sensor, v = vrep.simxGetVisionSensorImage(self.clientID,self.zedHandle,0,vrep.simx_opmode_blocking)
            ts = []
            for i in v:
                t = int2uint8(i)
                ts.append(t)

            t = np.array(ts)
            re = t.reshape(720,1280,3) # 变为三维矩阵
            re[:,:,[0,2]] = re[:,:,[2,0]] # opencv使用的是BGR，因此需要交换G和B
            mat = np.array(re,dtype=np.uint8)
            mat1 = cv2.flip(mat,0,dst=None) #垂直镜像
            self.image = mat1
    
    def findBlack(self): # 找二维码或者T、E
        mat = cv2.blur(self.image,(5,5)) # 模糊化降噪处理
        gray = cv2.cvtColor(mat,cv2.COLOR_BGR2GRAY)
        limit = 50 
        mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0），（存疑）达到凸显台子图案的目的


        # 先找到白色大块边界（去除边界黑块影响）
        # 利用黑色像素的位置得到中心坐标

        MAXX=0
        MINX=100000
        MAXY=0
        MINY=100000
        xlen = 1280
        ylen = 720

        for i in range(xlen): 
            for j in range(ylen):
                if mask[j][i] > 0: # 找白色
                    MAXX=max(MAXX,i)
                    MINX=min(MINX,i)
                    MAXY=max(MAXY,j)
                    MINY=min(MINY,j)
                    # 得到白色边界

        maxx=0
        minx=100000
        maxy=0
        miny=100000
        xlen = 1280
        ylen = 720

        for i in range(xlen): 
            for j in range(ylen):
                if mask[j][i] == 0 and i >= MINX and i <= MAXX and j >= MINY and j <= MAXY: # 找黑色
                    maxx=max(maxx,i)
                    minx=min(minx,i)
                    maxy=max(maxy,j)
                    miny=min(miny,j)

        # p = self.image[miny:maxy, minx:maxx]
        # cv2.imshow('111',p)
        # cv2.waitKey()

        self.targetX = (maxx + minx) / 2
        self.targetY = (maxy + miny) / 2
            

    def findTarget(self): # 找红色圆柱体target
        mat = cv2.blur(self.image,(5,5)) # 模糊化降噪处理
        im_hsv = cv2.cvtColor(mat,cv2.COLOR_BGR2HSV) # 转换为HSV

        lower_red = np.array([0,100,100])
        upper_red = np.array([10,255,255])
        red_mask = cv2.inRange(im_hsv,lower_red,upper_red) # 红色区域取255（白色），其余取0（黑色）
        

        binary = cv2.Canny(red_mask, 0, 60, apertureSize = 3)
        # cv2.imshow('binary.png',binary)
        # cv2.waitKey()
        contours, cnt = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) # 提取矩形轮廓

        rec = []
        for c in contours:
            x,y,w,h = cv2.boundingRect(c)#计算出一个简单地边界框
            if w < h : # 判断得到的矩形的大小是否符合（二维码的某个块即可）
                rec.append([x,y,w,h])

        mi = -1
        mw = 0
        l = len(rec)
        for i in range(l):
            x,y,w,h = rec[i]
            if (mw < w):
                mw = w
                mi = i
        if mi > -1:
            x,y,w,h = rec[mi]     

            # mmm = self.image[y:y+h, x:x+w]
            # cv2.imshow('mmm.png',mmm)
            # cv2.waitKey()

            self.targetX = x + w / 2
            self.targetY = y + h / 2
            return 1
        else:
            return 0

    def findQR(self): # 找二维码新方法
        limit = 50
        gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0）,达到凸显二维码的目的

        kernel = np.ones((5,5),np.uint8)
        erosion = cv2.erode(mask,kernel)
        erosion = cv2.erode(erosion,kernel)

        contours,hier=cv2.findContours(erosion, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        rec = []
        for c in contours:
            x,y,w,h = cv2.boundingRect(c)#计算出一个简单地边界框
            if (abs(w-h)<10) & (w>10): # 判断得到的矩形的大小是否符合（二维码的某个块即可）
                rec.append([x,y,w,h])

        mi = -1
        mw = 0
        l = len(rec)
        for i in range(l):
            x,y,w,h = rec[i]
            if (mw < w):
                mw = w
                mi = i
        if mi > -1:
            x,y,w,h = rec[mi]
            # mmm = self.image[y:y+h, x:x+w]
            # cv2.imshow('mmm.png',mmm)
            # cv2.waitKey()
            self.targetX = x + w / 2
            self.targetY = y + h / 2
            if w * self.pos[2] > 300:
                return 1 # 找到完整二维码
            else:
                return 2 # 找到二维码的一个块
        else:
            print('error!')
            return 0

    def findCircle(self):
        limit = 50
        gray = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray,limit,255) #将limit-255范围内的像素点全部转化为白色（255），0-limit为黑色（0）,达到凸显二维码的目的

        # cv2.imshow('mask',mask)
        # cv2.waitKey()

        # kernel = np.ones((5,5),np.uint8)
        # erosion = cv2.erode(mask,kernel)
        # erosion = cv2.erode(erosion,kernel)
        circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=30,minRadius=5,maxRadius=300)
        
        l = len(circles)
        m = -1
        rm = -1
        if l > 0:
            for i in range(l):
                x, y, r = circles[i][0]
                if rm < r:
                    rm = r
                    m = i
        
        if m > -1:
            x, y, r = circles[m][0]
            # mmm = self.image[int(y-r):int(y+r), int(x-r):int(x+r)]
            # cv2.imshow('mmm',mmm)
            # cv2.waitKey()
            self.targetX = x
            self.targetY = y
            return 1
        else:
            print('error')
            return 0

        

        

        