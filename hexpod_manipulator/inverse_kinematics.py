from operator import le
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import hexpod

class InverseKinematics:
  
  def LegLeangthSolver(self, Hexpod : hexpod.Hexpod, max_error = 0.1):
    # solve the position and rotation of the pad and the base
    base_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(Hexpod.base_leg_connections, Hexpod.base_translation, Hexpod.base_rotation)
    pad_leg_connection_3d_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(Hexpod.pad_leg_connections, Hexpod.pad_translation, Hexpod.pad_rotation)
    leg_length = np.linalg.norm(pad_leg_connection_3d_pos - base_leg_connection_3d_pos)
    Hexpod.leg_length = leg_length
    if leg_length > Hexpod.leg_max_length:
      print("Error: Leg length is too long", leg_length)
      return False
    if leg_length < Hexpod.leg_min_length:
      print("Error: Leg length is too short", leg_length)
      return False
    return True
    
    