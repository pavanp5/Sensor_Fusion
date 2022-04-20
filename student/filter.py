# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = 6 # process model dimension
        self.dt = 0.1 # time increment
        self.q=0.1 # process noise variable for Kalman filter Q
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        dt_l = self.dt
        return np.matrix([[1,0,0, dt_l,0,0],[0,1,0,0,dt_l,0],[0,0,1,0,0,dt_l],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])

        #return 0
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        a = (1/3)*((self.dt)**3)*(self.q)
        b = (1/2)*((self.dt)**2)*(self.q)
        c = (self.dt)*(self.q)
        return np.matrix([[0,0,0,a, 0, 0],[0,0,0,0, b, 0],[0,0,a,0,0,b],[0,0,b,c,0,0],[0,b,0,0,c,0],[0,0,b,0,0,c]])                                 
                                         

        #return 0
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        x = track.x
        P = track.P   
        
        F = self.F()
        Q = self.Q()
        x = np.matmul(F,x)
        P = F*P*(F.transpose()) + Q 
        track.set_x(x)
        track.set_P(P)
        return x, P                                 
        #pass
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        sensor = meas.sensor
        if sensor.name == 'lidar':
            print(sensor.name)
            x = track.x
            P = track.P
            print("Update Lidar")
            gamma = meas.z - meas.sensor.get_H(x)*x
            #print("gamma {}", gamma)
            H = meas.sensor.get_H(x)
            S = H*P*H.transpose() + meas.R
            K = P*H.transpose()*np.linalg.inv(S)
            x = x + K*gamma
            I = np.identity(self.dim_state)
            P = (I - K*H)*P
            #print("x {} P {}",x,P)
            track.set_x(x)
            track.set_P(P)
        elif sensor.name == "camera":
            print(sensor.name)
            x = track.x
            P = track.P
            print("Update Camera")
            gamma = meas.z - meas.sensor.get_hx(x)
            #print("gamma {}", gamma)
            H = meas.sensor.get_H(x)
            S = H*P*H.transpose() + meas.R
            K = P*H.transpose()*np.linalg.inv(S)
            x = x + K*gamma
            I = np.identity(self.dim_state)
            P = (I - K*H)*P
            #print("x {} P {}",x,P)
            track.set_x(x)
            track.set_P(P)
                   
        return x,P

        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        return 0
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return 0
        
        ############
        # END student code
        ############ 