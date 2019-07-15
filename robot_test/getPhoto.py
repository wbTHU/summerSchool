import vrep
import numpy as np
import math
import cv2
import time
import VisionSensor

Sensor = VisionSensor.VisionSensor
tstep = 0.0025

print ('program started!')
vrep.simxFinish(-1)

while True:
    clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
    if clientID != -1:
        break
    else:
        print ('Cannot connect to server!')
        time.sleep(0.05)

print ('Connection Success!')

vrep.simxSetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,tstep,vrep.simx_opmode_oneshot)
vrep.simxSynchronous(clientID,True)
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)

s = Sensor(clientID,'zed_vision0')


cv2.imwrite('9.png',s.image)


vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot)
