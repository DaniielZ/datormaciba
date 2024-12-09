import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import hexpod

class InverseKinematics:
  
  @staticmethod
  def LegLengthSolverEquation(inputs, pad_leg_connection, base_leg_connection, base_translation, base_rotation, pad_translation, pad_rotation):
    leg_length = inputs
    results = np.zeros(6)
    # calculate the 3d pos of the base and pad leg connections
    base_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(base_leg_connection, base_translation, base_rotation)
    pad_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(pad_leg_connection, pad_translation, pad_rotation)
    
    for i in range(6):
      results[i] = np.linalg.norm(pad_leg_connection_3d_pos[i] - base_leg_connection_3d_pos[i]) - leg_length[i]
    return results
  
  def LegLeangthSolver(self, Hexpod, max_error = 0.1):
    # solve the position and rotation of the pad and the base
    bounds = (Hexpod.leg_min_length*np.ones(6), Hexpod.leg_max_length*np.ones(6))
    result = least_squares(self.LegLengthSolverEquation, Hexpod.leg_length, 
                           args=(Hexpod.pad_leg_connections, 
                                 Hexpod.base_leg_connections, 
                                 Hexpod.base_translation, 
                                 Hexpod.base_rotation, 
                                 Hexpod.pad_translation, 
                                 Hexpod.pad_rotation), 
                           max_nfev=100000, ftol=1e-12, xtol=1e-12, gtol=1e-12, bounds=bounds)
    
    max_deviation = np.max(np.abs(result.fun))
    
    if max_deviation > max_error:
        print("Error: Could not solve for leg lengths")
        print("Max deviation: ", max_deviation)
        print("Result: ", result.x)
        print("Message: ", result.message)
        print("Status: ", result.status) 
        Hexpod.leg_length = result.x
        return False
    else:
        Hexpod.leg_length = result.x
        return True