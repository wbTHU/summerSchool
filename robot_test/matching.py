from os.path import join
from os import walk
import numpy as np
import cv2
def match(target,picture):
    # 获取训练好的人脸的参数数据，这里直接从GitHub上使用默认值
    face_cascade = cv2.CascadeClassifier(r'D:/wurenji/summerSchool-fan_v_rep_branch/robot_test/haarcascade_frontalface_default.xml')

    # 读取图片
    image = cv2.imread(picture)
    #cv2.imshow("Find Faces!",image)
    #image2=cv2.resize(image, (120,120), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    # 探测图片中的人脸
    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor = 1.15,
        minNeighbors = 5,
        minSize = (5,5),
        flags = cv2.CASCADE_SCALE_IMAGE
    )

    #print ("发现{0}个人脸!".format(len(faces)))
    xx=0
    yy=0
    ww=0
    hh=0
    max_matches=0
    i=0
    for(x,y,w,h) in faces:


        subimage=image[y:y+w,x:x+w]

        #cv2.imwrite("D:/wurenji/summerSchool-fan_v_rep_branch/people_img/"+str(i)+".png",subimage)
        #cv2.waitKey(5000)
        # 使用SIFT算法检查图像的关键点和描述符
        sift = cv2.xfeatures2d.SIFT_create()
        query_kp, query_ds = sift.detectAndCompute(subimage, None)

        # 创建FLANN匹配器
        index_params = dict(algorithm=0, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        potential_culprits = {}
        
        # 将图像query与特征数据文件的数据进行匹配
        matches = flann.knnMatch(query_ds, np.load(target), k=2)
        # 清除错误匹配
        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        # 输出每张图片与目标图片的匹配数目
        #print( len(good))
        if len(good)>max_matches:
            xx=x
            yy=y
            ww=w
            hh=h
            max_matches=len(good)
    return [int((xx+ww)/2),int((yy+ww)/2)]


def match_alpha(img):


#载入并显示图片
    
    #cv2.imshow('img',img)
    #灰度化
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #输出图像大小，方便根据图像大小调节minRadius和maxRadius
    #print(img.shape)
    #霍夫变换圆检测
    circles= cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,100,param1=100,param2=30,minRadius=5,maxRadius=300)
    #输出返回值，方便查看类型

    #输出检测到圆的个数
    #print(len(circles[0]))
    #根据检测到圆的信息，画出每一个圆
    for circle in circles[0]:
        #圆的基本信息
       
        #坐标行列
        x=int(circle[0])
        y=int(circle[1])
        #半径
        r=int(circle[2])
        #在原图用指定颜色标记出圆的位置
        img=cv2.circle(img,(x,y),r,(0,0,255),-1)
    #cv2.imshow('res',img)
    #显示新图像
    return circles

