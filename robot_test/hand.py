import vrep
import numpy as np
import math
import time
import ctypes

class Hand():
    
    def __init__(self, clientID, handSignal):
        self.clientID = clientID
        self.handSignal = handSignal
    
    def close(self):
        vrep.simxSetStringSignal(self.clientID,self.handSignal,'1',vrep.simx_opmode_blocking) 
        vrep.simxSynchronousTrigger(self.clientID)

    
    def open(self):
        vrep.simxSetStringSignal(self.clientID,self.handSignal,'0',vrep.simx_opmode_blocking) 
        vrep.simxSynchronousTrigger(self.clientID)




    
    
