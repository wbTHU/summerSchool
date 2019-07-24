import cv2
import numpy as np 


frame = cv2.imread('0.png')



x,y,w,h = 610,320,40,80
track_window = (x,y,w,h)

r = frame[y:y+h, x:x+w]
cv2.imshow('r',r)
cv2.waitKey()

h = cv2.cvtColor(r,cv2.COLOR_BGR2HSV)
mask = cv2.inRange(h,np.array((0., 60.,32.)), np.array((180.,255.,255.)))
roi_hist = cv2.calcHist([h],[0],mask,[180],[0,180])
cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 1 )
i = 5
while(i > 0):
    frame = cv2.imread(str(5 - i) + '.png')
    i = i - 1
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

    ret, track_window = cv2.CamShift(dst, track_window, term_crit)
    print(track_window)

    pts = cv2.boxPoints(ret)
    pts = np.int0(pts)
    img2 = cv2.polylines(frame,[pts],True, 255,2)

    cv2.imshow('img2',img2)
    cv2.waitKey()

    x,y,w,h = track_window
    tx = x + w / 2
    ty = y + h / 2
    print('tx is ' + str(tx))
    print('ty is ' + str(ty))

    r = frame[y:y+h, x:x+w]
    cv2.imshow('r',r)
    cv2.waitKey()