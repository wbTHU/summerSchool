import vrep
import numpy as np
import math
import time
import ctypes


tstep = 0.005
baseName = 'Quadricopter_base'
handName = 'JacoHand'
handSignal = 'close_hand'





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


print('Handles available!')



vrep.simxSynchronousTrigger(clientID)
vrep.simxGetPingTime(clientID) 


if vrep.simxGetConnectionId(clientID) != -1:
    
    vrep.simxSetStringSignal(clientID,'goto_close','1',vrep.simx_opmode_oneshot) #'1':合上手  '0':张开手
    
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID) 



    
    
