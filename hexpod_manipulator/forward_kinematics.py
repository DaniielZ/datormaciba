# goal is to have a class that takes in the hexpod object and calculated the 6Dof of the top given base and legs

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve
import hexpod



class ForwardKinematics:

  def PadPosRotSolverEquation(self, inputs, pad_leg_connection, base_leg_connection, base_translation, base_rotation, leg_length):
    pad_translation = inputs[0:3]
    pad_rotation = inputs[3:6]
    results = np.zeros(6)
    if (pad_rotation[0] > np.pi/2) or (pad_rotation[0] < -np.pi/2):
      return np.ones(6)*np.inf
    if (pad_rotation[1] > np.pi/2) or (pad_rotation[1] < -np.pi/2):
      return np.ones(6)*np.inf
    
    # calculate the 3d pos of the base and pad leg connections
    base_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(base_leg_connection, base_translation, base_rotation)
    pad_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(pad_leg_connection, pad_translation, pad_rotation)
    
    for i in range(6):
      results[i] = np.linalg.norm(pad_leg_connection_3d_pos[i] - base_leg_connection_3d_pos[i]) - leg_length[i]
    return results
    

  def PadPosRotSolver(self, Hexpod, max_error = 0.1):
    # solve the position and rotation of the pad and the base
    #limit pad rotation
    
    solution, infodict, ier, msg = fsolve(self.PadPosRotSolverEquation, np.zeros(6), 
                                          args=(Hexpod.pad_leg_connections, 
                                                Hexpod.base_leg_connections, 
                                                Hexpod.base_translation, 
                                                Hexpod.base_rotation, 
                                                Hexpod.leg_length), full_output=True)
    
    max_deviation = np.max(np.abs(infodict['fvec']))
    
    if max_deviation > max_error:
        print("Error: Could not solve for pad position and rotation")
        print("Max deviation: ", max_deviation)
        print("Result: ", solution)
        return False
    else:
        Hexpod.pad_translation = solution[0:3]
        Hexpod.pad_rotation = solution[3:6]
        return True

