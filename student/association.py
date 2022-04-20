# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

import misc.params as params 

class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, track_list, meas_list, KF):
             
        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based on Mahalanobis distance (see below) for all tracks and all measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        # the following only works for at most one track and one measurement
        self.association_matrix = np.matrix([]) # reset matrix
        self.unassigned_tracks = [] # reset lists
        self.unassigned_meas = []
        
        if len(meas_list) > 0:
            self.unassigned_meas = [0]
        if len(track_list) > 0:
            self.unassigned_tracks = [0]
        if len(meas_list) > 0 and len(track_list) > 0: 
            self.association_matrix = np.matrix([[0]])
        
        M=len(track_list)
        N=len(meas_list)
        self.unassigned_tracks= list(range(M)) #track_list
        self.unassigned_meas = list(range(N)) #meas_list
        
        self.association_matrix = np.inf*np.ones((len(track_list),len(meas_list)))
        
        for i in range(M):
            for j in range(N):
                #print("sensor name{}, x{},y{}",meas_list[j].sensor.name,meas_list[j].z[0],meas_list[j].z[1])
                dist = self.MHD(track_list[i],meas_list[j],KF)
                if self.gating(dist):
                    self.association_matrix[i,j]=dist
        
        ############
        # END student code
        ############ 
                
    def get_closest_track_and_meas(self):
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks and unassigned_meas
        # - return this track and measurement
        ############

        track_list=self.unassigned_tracks
        meas_list=self.unassigned_meas
             
        if (np.argmin(self.association_matrix) != np.inf):
            
            ij_min=np.unravel_index(np.argmin(self.association_matrix),self.association_matrix.shape)
            i = track_list[ij_min[0]]
            j = meas_list[ij_min[1]]
            self.unassigned_tracks.remove(track_list[ij_min[0]])
            self.unassigned_meas.remove(meas_list[ij_min[1]])
            self.association_matrix=np.delete(self.association_matrix,ij_min[0],0)
            self.association_matrix=np.delete(self.association_matrix,ij_min[1],1)
            
        else:
            i=np.nan
            j=np.nan        
        update_track=i
        update_meas=j
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self, MHD): 
        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        limit=chi2.ppf(params.gating_threshold, df=params.dim_state)

        if MHD<limit:
            return True
        else:
            return False
        #pass    
        
        ############
        # END student code
        ############ 
        
    def MHD(self, track, meas, KF):
        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############

        H = meas.sensor.get_H(track.x)
        gamma = meas.z - meas.sensor.get_hx(track.x)
        S = H*track.P*H.transpose() + meas.R
        MHD = gamma.transpose()*np.linalg.inv(S)*gamma
        
        #pass
        return MHD
        
        ############
        # END student code
        ############ 
    
    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)
    
        # update associated tracks with measurements
        while self.association_matrix.shape[0]>0 and self.association_matrix.shape[1]>0:
            
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]
            
            # check visibility, only update tracks in fov    
            if not meas_list[0].sensor.in_fov(track.x):
                track.set_idle(track.idle+1)
                continue
            
            track.set_idle(0)
            # Kalman update
            print('update track', track.id, 'with', meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])
            
            # update score and track state 
            manager.handle_updated_track(track)
            
            # save updated track
            manager.track_list[ind_track] = track
            
        # run track management 
        manager.manage_tracks(self.unassigned_tracks, self.unassigned_meas, meas_list)
        
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)