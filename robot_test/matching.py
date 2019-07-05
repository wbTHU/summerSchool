from os.path import join
from os import walk
import numpy as np
import cv2
def match(query,path):
    
    #folder = 'D:/wurenji/summerSchool-fan_v_rep_branch/robot_test/people_img'
    folder = path
    descriptors = []
    # 获取特征数据文件名
    for (dirpath, dirnames, filenames) in walk(folder):
        for f in filenames:
            if f.endswith("npy"):
                descriptors.append(f)
        #print(descriptors)

    # 使用SIFT算法检查图像的关键点和描述符
    sift = cv2.xfeatures2d.SIFT_create()
    query_kp, query_ds = sift.detectAndCompute(query, None)

    # 创建FLANN匹配器
    index_params = dict(algorithm=0, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    potential_culprits = {}
    for d in descriptors:
        # 将图像query与特征数据文件的数据进行匹配
        matches = flann.knnMatch(query_ds, np.load(join(folder, d)), k=2)
        # 清除错误匹配
        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)
        # 输出每张图片与目标图片的匹配数目
        #print("img is %s ! matching rate is (%d)" % (d, len(good)))
        potential_culprits[d] = len(good)

    # 获取最多匹配数目的图片
    max_matches = None
    potential_suspect = None
    for culprit, matches in potential_culprits.items():
        
        if max_matches == None or matches > max_matches:
            max_matches = matches
            potential_suspect = culprit

    #print("potential suspect is %s" % potential_suspect.replace("npy", ""))
    return potential_suspect.replace("npy", "")
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

