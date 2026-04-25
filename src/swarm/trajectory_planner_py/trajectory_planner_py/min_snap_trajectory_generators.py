# -*- coding: utf-8 -*-
# optimizeTrajectory taken from traj_gen library https://github.com/icsl-Jeon/traj_gen

import time
from typing import Tuple
import numpy as np
from numpy import pi
from .linear_type import min_deriv_poly, polynom
from .optimize_type import PolyTrajGen

class YawMinAccTrajectory:

    def __init__(self, yaw_waypoints:np.array, t_waypoints:np.array):
        
        self.deriv_order = 2 # acceleration
        self.nb_coeff = self.deriv_order*2
        self.wps   = np.copy(yaw_waypoints)
        self.t_wps   = np.copy(t_waypoints)
        self.t_segment = np.diff(self.t_wps)
        self.t_idx = 0  # index of current segment
        
        # check dimensions and values are good
        if len(self.t_wps) != self.wps.shape[0]:
            raise Exception("Time array and waypoint array not the same size.")
        elif (np.diff(self.t_wps) <= 0).any():
            raise Exception("Time array isn't properly ordered.")  

        self.coeffs = min_deriv_poly(self.wps, self.t_segment, self.deriv_order)

        # initialize the values to zeros
        self.des_yaw = 0.0    # Desired yaw
        self.des_yaw_rate = 0.0    # Desired yaw rate
        self.des_yaw_acc = 0.0    # Desired yaw angular acceleration

        self.prev_time = 0.0
        self.first = True

    def  eval(self, t:float, des_pos:np.array=np.zeros(3), curr_pos:np.array=np.zeros(3))->Tuple[float, float, float]:
        
        if self.first:
            self.prev_time = t
        dt = t - self.prev_time
        
        # initialize the values to zeros
        self.des_yaw = 0.0    # Desired yaw
        self.des_yaw_rate = 0.0    # Desired yaw rate
        self.des_yaw_acc = 0.0    # Desired yaw angular acceleration

        # use first waypoint at the beginning
        if t == 0:
            self.t_idx = 0
            self.des_yaw = self.wps[0]
        # Stay hover at the last waypoint position
        else:
            # find which time segment we are at
            self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
            # Scaled time (between 0 and duration of segment)
            scale = (t - self.t_wps[self.t_idx])
            
            # ==== Calculate yaw, yaw rate, angular acceleration at time t ====
            # Which coefficients to use
            start = self.nb_coeff * self.t_idx
            end = self.nb_coeff * (self.t_idx + 1)
            
            # desired yaw
            t0 = polynom(self.nb_coeff, 0, scale)
            self.des_yaw = self.coeffs[start:end].dot(t0)
            # desired yaw rate
            t1 = polynom(self.nb_coeff, 1, scale)
            self.des_yaw_rate = self.coeffs[start:end].dot(t1)
            # desired yaw angular acceleration
            t2 = polynom(self.nb_coeff, 2, scale)
            self.des_yaw_acc = self.coeffs[start:end].dot(t2)
            #================

        return self.des_yaw, self.des_yaw_rate, self.des_yaw_acc

class optimizeTrajectory:
    def __init__(self, xyz_waypoints:np.array, t_waypoints:np.array, optim_target:str,poly_order:int=7, floating_cubes:np.array=None, t_cubes:np.array=None):
        
        self.dim = 3
        self.wps   = np.copy(xyz_waypoints)
        self.t_wps   = np.copy(t_waypoints)
        self.optim_target = optim_target #'end-derivative' 'poly-coeff'

        # check dimensions and values are good
        if len(self.t_wps) != self.wps.shape[0]:
            raise Exception("Time array and waypoint array not the same size.")
        elif (np.diff(self.t_wps) <= 0).any():
            raise Exception("Time array isn't properly ordered.")  
        
        max_continuous_deriv = 4
        objWeights = np.array([0, 0, 0, 1])    
        self.traj_gen = PolyTrajGen(self.t_wps, poly_order, self.optim_target, self.dim, max_continuous_deriv)

        Xdot = np.array([0, 0, 0])
        Xddot = np.array([0, 0, 0])

        # add waypoints
        for i in range(self.wps.shape[0]):
            # create pin dictionary
            pin_ = {'t':self.t_wps[i], 'd':0, 'X':self.wps[i]}
            self.traj_gen.addPin(pin_)
        
        # add velocity & acceleration constraints for first waypoint (vel=0, acc=0)
        pin_ = {'t':self.t_wps[0], 'd':1, 'X':Xdot}
        self.traj_gen.addPin(pin_)
        pin_ = {'t':self.t_wps[0], 'd':2, 'X':Xddot,}
        self.traj_gen.addPin(pin_)

        # add velocity & acceleration constraints for last waypoint (vel=0, acc=0)
        pin_ = {'t':self.t_wps[-1], 'd':1, 'X':Xdot}
        self.traj_gen.addPin(pin_)
        pin_ = {'t':self.t_wps[-1], 'd':2, 'X':Xddot,}
        self.traj_gen.addPin(pin_)
        
        # Add passthrough waypoints if provided. These are used to shape the trajectory and ensure it passes through them.
        if floating_cubes is not None and t_cubes is not None and len(floating_cubes)==len(t_cubes):
            for pass_cube, t_cube in zip(floating_cubes, t_cubes):
                # each cube has dimension of 3*2
                pin_ = {'t':t_cube, 'd':0, 'X':pass_cube}
        self.traj_gen.addPin(pin_)

        # solve
        self.traj_gen.setDerivativeObj(objWeights)
        print("solving trajectory optimization")
        time_start = time.time()
        self.traj_gen.solve()
        time_end = time.time()
        print(f"completed optimization in {time_end - time_start}")
    
    
    def  eval(self, t:float)->Tuple[np.array, float, np.array, np.array, np.array, np.array]:
        
        position = np.zeros(3)    # Desired position (x, y, z)
        vel = np.zeros(3)    # Desired velocity (xdot, ydot, zdot)
        acc = np.zeros(3)    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        jerk = np.zeros(3)    # Desired jerk (xdotdotdot, ydotdotdot, zdotdotdot)
        snap = np.zeros(3)    # Desired snap (xdotdotdotdot, ydotdotdotdot, zdotdotdotdot)
   
       # use first waypoint at the beginning
        if t == 0:
            t_idx = 0
            position = self.wps[t_idx,:]
        # Stay hover at the last waypoint position
        elif (t >= self.t_wps[-1]):
            t_idx = -1
            position = self.wps[t_idx,:]
        else:            
            position = self.traj_gen.eval(np.array([t]), 0).reshape(3)
            vel = self.traj_gen.eval(np.array([t]), 1).reshape(3)
            acc = self.traj_gen.eval(np.array([t]), 2).reshape(3)
            jerk = self.traj_gen.eval(np.array([t]), 3).reshape(3)
            snap = self.traj_gen.eval(np.array([t]), 4).reshape(3)
        return position, vel, acc, jerk, snap
    