#!/usr/bin/env python
# coding=utf-8

import numpy as np
from abc import abstractmethod
from matplotlib import pyplot as plt
from matplotlib import rc
import matplotlib.patches as patches

class TrajGen(object):
    def __init__(self, knots_, dim_,):
        # dimension of curve
        self.dim = dim_
        # time knots
        self.Ts = knots_
        # set of pin
        self.pinSet = None
        # is solved?
        self.isSolved = False
        # time nkots (length = M for polyTrajGen and length 2 for optimalTraj)
        self.weight_mask = None
        # pin sets
        ## fixed
        self.fixPinSet = {}
        self.loosePinSet = {}
        self.fixPinOrder = {}

    @abstractmethod
    def setDerivativeObj(self, weight):
        pass

    @abstractmethod
    def solve(self,):
        pass

    @abstractmethod
    def eval(self, t, d):
        pass

    def addPin(self, pin):
        assert pin['X'].shape[0] == self.dim, "dim of pin val != dim of this TrajGen"
        assert (pin['t']>=self.Ts[0] and pin['t']<=self.Ts[-1]), 't of this pin is out of range of knots'
        if self.pinSet is not None:
            self.pinSet.append(pin)
        else:
            self.pinSet = [pin]

    def addPinSet(self, pinSet_):
        for pin in pinSet_:
            self.addPin(pin)


