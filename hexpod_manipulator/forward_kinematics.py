# goal is to have a class that takes in the hexpod object and calculated the 6Dof of the top given base and legs

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import hexpod



class ForwardKinematics:
  def __init__(self):
    return

  def PadPosRotSolverEquation(self, inputs, pad_leg_connection, base_leg_connection, base_translation, base_rotation, leg_length):
    pad_translation = inputs[0:3]
    pad_rotation = inputs[3:6]
    results = np.zeros(6)
    # if (pad_rotation[0] > np.pi/2) or (pad_rotation[0] < -np.pi/2):
    #   return np.ones(6)*np.inf
    # if (pad_rotation[1] > np.pi/2) or (pad_rotation[1] < -np.pi/2):
    #   return np.ones(6)*np.inf
    # if (pad_rotation[2] >  (np.deg2rad(45) + np.pi/4)) or (pad_rotation[2] < ( np.deg2rad(45)-np.pi/4)):
    #   return np.ones(6)*np.inf
    
    # calculate the 3d pos of the base and pad leg connections
    base_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(base_leg_connection, base_translation, base_rotation)
    pad_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(pad_leg_connection, pad_translation, pad_rotation)
    
    for i in range(6):
      results[i] = np.linalg.norm(pad_leg_connection_3d_pos[i] - base_leg_connection_3d_pos[i]) - leg_length[i]
    return results
    

  def PadPosRotSolver(self, Hexpod, max_error = 0.1):
    # solve the position and rotation of the pad and the base
    bounds = ([-np.inf, -np.inf, -np.inf, -np.pi/2, -np.pi/2, (np.deg2rad(45)-np.pi/4)], 
              [np.inf, np.inf, np.inf, np.pi/2, np.pi/2, (np.deg2rad(45)+np.pi/4)])
    result = least_squares(self.PadPosRotSolverEquation, np.zeros(6), 
                           args=(Hexpod.pad_leg_connections, 
                                 Hexpod.base_leg_connections, 
                                 Hexpod.base_translation, 
                                 Hexpod.base_rotation, 
                                 Hexpod.leg_length), 
                           max_nfev=100000, ftol=1e-12, xtol=1e-12, gtol=1e-12, bounds=bounds)
    
    max_deviation = np.max(np.abs(result.fun))
    
    if max_deviation > max_error:
        print("Error: Could not solve for pad position and rotation")
        print("Max deviation: ", max_deviation)
        print("Result: ", result.x)
        print("Message: ", result.message)
        print("Status: ", result.status) 
        Hexpod.pad_translation = result.x[0:3]
        Hexpod.pad_rotation = result.x[3:6]
        return False
    else:
        Hexpod.pad_translation = result.x[0:3]
        Hexpod.pad_rotation = result.x[3:6]
        return True