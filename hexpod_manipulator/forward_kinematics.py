# goal is to have a class that takes in the hexpod object and calculated the 6Dof of the top given base and legs

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve



class ForwardKinematics:

  def PadPosRotSolverEquation(pad_trans_rot, pad_leg_connection, base_leg_connection, base_translation, base_rotation, leg_length):
    pad_translation = pad_trans_rot[0:3]
    pad_rotation = pad_trans_rot[3:6]
    results = np.zeros(6)
    pad_leg_connection_3d_pos = pad_leg_connection*R.from_euler(pad_rotation) + pad_translation
    base_leg_connection_3d_pos = base_leg_connection*R.from_euler(base_rotation) + base_translation
    for i in range(6):
      results[i] = np.linalg.norm(pad_leg_connection_3d_pos[i] - base_leg_connection_3d_pos[i]) - leg_length[i]
    return results
    

  def PadPosRotSolver(self, Hexpod, max_error = 0.1):
    # solve the position and rotation of the pad and the base
    
    result = fsolve(self.PadPosRotSolverEquation, np.zeros(6), args=(Hexpod.pad_leg_connections, Hexpod.base_leg_connections, Hexpod.base_translation, Hexpod.base_rotation, Hexpod.leg_length), full_output=True)
    max_deviation = np.max(np.abs(result.infodict['fvec']))
    
    if max_deviation > max_error:
      print("Error: Could not solve for pad position and rotation")
      print("Max deviation: ", max_deviation)
      print("Result: ", result[0:6])
      return False
    else:
      Hexpod.pad_translation = result[0:3]
      Hexpod.pad_rotation = result[3:6]
      return True

