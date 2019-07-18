#-*- coding:utf-8 -*- 

import numpy as np
import vrep
import ctypes
import math
import sys
import time
import cv2
import PIL
import matplotlib.pyplot as plt

sim_dt = 0.01 
dt = 0.001


SYNC = True
vrep_mode = vrep.simx_opmode_oneshot

def b( num ):
    """ forces magnitude to be 1 or less """
    if abs( num ) > 1.0:
        return math.copysign( 1.0, num )
    else:
        return num

def convert_angles( ang ):
    """ Converts Euler angles from x-y-z to z-x-y convention """
    s1 = math.sin(ang[0])
    s2 = math.sin(ang[1])
    s3 = math.sin(ang[2])
    c1 = math.cos(ang[0])
    c2 = math.cos(ang[1])
    c3 = math.cos(ang[2])

    pitch = math.asin( b(c1*c3*s2-s1*s3) )
    cp = math.cos(pitch)
    # just in case
    if cp == 0:
        cp = 0.000001

    yaw = math.asin( b((c1*s3+c3*s1*s2)/cp) ) #flipped
    # Fix for getting the quadrants right
    if c3 < 0 and yaw > 0:
        yaw = math.pi - yaw
    elif c3 < 0 and yaw < 0:
        yaw = -math.pi - yaw
    
    roll = math.asin( b((c3*s1+c1*s2*s3)/cp) ) #flipped
    return [roll, pitch, yaw]

class Quadcopter( object ):
    """
    This callable class will return the state of the quadcopter relative to its
    target whenever it is called. It will also accept motor commands which will be
    sent to the quadcopter in V-REP.
    """
    def __init__( self, max_target_distance=4, noise=False,
                  noise_std=None, dodging=True,
                  target_func=None, cid=None
                ):

        # If a cid is specified, assume the connection has already been
        # established and should remain open
        if cid is None:
            vrep.simxFinish(-1) # just in case, close all opened connections
            self.cid = vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
        else:
            self.cid = cid

        if self.cid != -1:
            print ('Connected to V-REP remote API server, client id: %s' % self.cid)
            vrep.simxStartSimulation( self.cid, vrep.simx_opmode_oneshot )
            if SYNC:
                vrep.simxSynchronous( self.cid, True )
        else:
            print ('Failed connecting to V-REP remote API server')
            self.exit()

        err, self.copter = vrep.simxGetObjectHandle(self.cid, "Quadricopter_base",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.target = vrep.simxGetObjectHandle(self.cid, "Quadricopter_target",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.laser_signal = vrep.simxReadStringStream(self.cid, 'laser_data', vrep.simx_opmode_streaming)
        err, self.dis_signal = vrep.simxReadStringStream(self.cid, 'dis_data', vrep.simx_opmode_streaming)
        
        # Reset the motor commands to zero
        packedData=vrep.simxPackFloats([0,0,0,0])
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        
        err = vrep.simxSetStringSignal(self.cid, "rotorTargetVelocities",
                                        raw_bytes,
                                        vrep_mode)
    
        self.pos = [0,0,0]
        self.pos_err = [0,0,0]
        self.t_pos = [0,0,0]
        self.lin = [0,0,0]
        self.ori = [0,0,0]
        self.ori_err = [0,0,0]
        self.t_ori = [0,0,0]
        self.ang = [0,0,0]
        self.count = 0
        self.first = True
        self.figure = plt.figure()
        self.x_min = 999
        self.y_min = 999  #min distance to wall

        # Maximum target distance error that can be returned
        self.max_target_distance = max_target_distance
        # If noise is being modelled
        if noise_std is not None:
            self.noise = True
        else:
            self.noise = False

        # Standard Deviation of the noise for the 4 state variables
        self.noise_std = noise_std
        # Overwrite the get_target method if the target is to be controlled by a
        # function instead of by V-REP
        # if target_func is not None:
          
        #     self.step = 0
        #     self.target_func = target_func

        #     def get_target():
        #         err, t_ori = vrep.simxGetObjectOrientation(self.cid, self.target, -1,
        #                                             vrep_mode )
        #         err, t_pos = vrep.simxGetObjectPosition(self.cid, self.target, -1,
        #                                         vrep_mode )
        
        #         # Convert orientations to z-y-x convention
        #         t_ori = convert_angles(t_ori)
        #         self.t_pos, self.t_ori = self.target_func( self.step )
        #         self.step += 1

        #     self.get_target = get_target
  
    def stop( self ):
        """
        Stops the simulation
        """
        err = vrep.simxStopSimulation( self.cid, vrep.simx_opmode_oneshot_wait )
        time.sleep(0.01) # Maybe this will prevent V-REP from crashing as often
        return hasattr(self, 'failed') # Returns true if this is a failed run


    def reset( self ):
        err = vrep.simxStopSimulation(self.cid, vrep.simx_opmode_oneshot_wait)
        time.sleep(1)
        self.pos_err = [0,0,0]
        self.ori_err = [0,0,0]
        self.lin = [0,0,0]
        self.ang = [0,0,0]
        err = vrep.simxStartSimulation(self.cid, vrep.simx_opmode_oneshot_wait)
        if SYNC:
            vrep.simxSynchronous( self.cid, True )
    
    def exit( self ):
        self.failed = True
        exit(1)

    # def get_target( self ):
    #     err, self.t_ori = vrep.simxGetObjectOrientation(self.cid, self.target, -1,
    #                                                 vrep_mode )
    #     err, self.t_pos = vrep.simxGetObjectPosition(self.cid, self.target, -1,
    #                                             vrep_mode )
        
    #     # Convert orientations to z-y-x convention
    #     self.t_ori = convert_angles(self.t_ori)

    def get_target(self, next_pos, next_ori):
        
        for i in range(3):
            self.t_pos[i] += next_pos[i]
            self.t_ori[i] += next_ori[i]

    def send_target_pos(self):  #bad deal
        # print('pos is ', self.t_pos)
        vrep.simxSetObjectPosition(self.cid, self.target, -1, self.t_pos, vrep.simx_opmode_oneshot)
        # vrep.simxSetObjectOrientation(self.cid, self.target, -1, self.t_ori, vrep.simx_opmode_oneshot)
        

    def calculate_error( self ):
        # Return the state variables
        err, self.ori = vrep.simxGetObjectOrientation(self.cid, self.copter, -1,
                                                vrep_mode )
        err, self.pos = vrep.simxGetObjectPosition(self.cid, self.copter, -1,
                                            vrep_mode )
        err, self.lin, self.ang = vrep.simxGetObjectVelocity(self.cid, self.copter,
                                                            vrep_mode )
        
        self.ori = convert_angles(self.ori)
        
        # Apply noise to each measurement if required
        #FIXME this is a dumb way to do this, clean it up later
        if self.noise:
            n_pos = np.random.normal(0,self.noise_std[0],3)
            n_lin = np.random.normal(0,self.noise_std[1],3)
            n_ori = np.random.normal(0,self.noise_std[2],3)
            n_ang = np.random.normal(0,self.noise_std[3],3)
            for i in range(3):
                self.pos[i] += n_pos[i]
                self.lin[i] += n_lin[i]
                self.ori[i] += n_ori[i]
                self.ang[i] += n_ang[i]
            #TODO: might have to wrap angles here
        
        # Find the error
        self.ori_err = [self.t_ori[0] - self.ori[0], 
                        self.t_ori[1] - self.ori[1],
                        self.t_ori[2] - self.ori[2]]
        cz = math.cos(self.ori[2])
        sz = math.sin(self.ori[2])
        x_err = self.t_pos[0] - self.pos[0]
        y_err = self.t_pos[1] - self.pos[1]
        self.pos_err = [ x_err * cz + y_err * sz, 
                        -x_err * sz + y_err * cz, 
                         self.t_pos[2] - self.pos[2]]
        
        self.lin = [self.lin[0]*cz+self.lin[1]*sz, -self.lin[0]*sz+self.lin[1]*cz, self.lin[2]]
        self.ang = [self.ang[0]*cz+self.ang[1]*sz, -self.ang[0]*sz+self.ang[1]*cz, self.ang[2]]

        for i in range(3):
            if self.ori_err[i] > math.pi:
                self.ori_err[i] -= 2 * math.pi
            elif self.ori_err[i] < -math.pi:
                self.ori_err[i] += 2 * math.pi

    def send_motor_commands( self, values ):

        # Limit motors by max and min values
        motor_values = np.zeros(4)
        for i in range(4):
            motor_values[i] = values[i]
        packedData=vrep.simxPackFloats(motor_values.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        err = vrep.simxSetStringSignal(self.cid, "rotorTargetVelocities",
                                        raw_bytes,
                                        vrep_mode)

    #----------------   get laser data and draw map  -------------------#
    def draw_map(self, obj):
        x = []
        y = []
        img_np = []
        minx, miny = 999, 999
        err, self.laser_signal = vrep.simxReadStringStream(self.cid, 'laser_data', vrep.simx_opmode_buffer)
        
        if err == vrep.simx_return_ok:
            laser_data = vrep.simxUnpackFloats(self.laser_signal)
            for i in range(0, len(laser_data), 3):
                if laser_data[i + 2 ] > 0:  #remove floor data
                    x.append(laser_data[i])
                    y.append(laser_data[i + 1])
            if len(x) != 0 and self.first:
                img_np = self.draw_plt(x, y)     

            if len(img_np) != 0:
                obj.send_data(img_np)
                
        
            
    def draw_plt(self, x, y):
        self.figure.patch.set_facecolor('black')
        plt.xlim(xmax = 6, xmin = -6)
        plt.ylim(ymax = 6, ymin = -6)
        plt.axis('off')
        plt.gca().xaxis.set_major_locator(plt.NullLocator())
        plt.gca().yaxis.set_major_locator(plt.NullLocator())
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
        plt.margins(0,0)
        plt.plot(x, y, 'ro')
       
        im = self.fig2img( self.figure )
        w, h = im.size
        im = np.array(im)
        res = cv2.resize(im, dsize=(math.ceil(w / 8), math.ceil(h / 8)), interpolation=cv2.INTER_CUBIC)
        self.figure.clear()
        return res
    
    def fig2data (self, fig):
        """
        @brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
        @param fig a matplotlib figure
        @return a numpy 3D array of RGBA values
        """
        # draw the renderer
        fig.canvas.draw ( )
    
        # Get the RGBA buffer from the figure
        w,h = fig.canvas.get_width_height()
        buf = np.fromstring ( fig.canvas.tostring_argb(), dtype=np.uint8 )
        buf.shape = ( w, h,4 )
    
        # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
        buf = np.roll ( buf, 3, axis = 2 )
        return buf

    def fig2img (self, fig):
        """
        @brief Convert a Matplotlib figure to a PIL Image in RGBA format and return it
        @param fig a matplotlib figure
        @return a Python Imaging Library ( PIL ) image
        """
        # put the figure pixmap into a numpy array
        buf = self.fig2data ( fig )
        w, h, d = buf.shape
        return PIL.Image.frombytes( "RGBA", ( w ,h ), buf.tostring( ) )

