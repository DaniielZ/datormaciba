from unittest import result

from attr import s
import hexpod 
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# This solver assumes that the whole pad is a rigid body
class ForceSolver:
  
  forces = np.zeros(6)
  def __init__(self):
    return
  
  @staticmethod
  def TranslateReactiveForces3D(forces, pad_leg_connections_3d, base_leg_connections_3d):
    results = np.zeros((6,3))
    for i in range(6):
      vector = pad_leg_connections_3d[i] - base_leg_connections_3d[i]
      results[i] = forces[i] * (vector/ np.linalg.norm(vector))
    return results
  
  @staticmethod
  def ForceSolverEquation(forces, pad_leg_connections_3d, base_leg_connections_3d, force, force_postion):
    
    translation_sum = force #force of weight on pad
    moment_sum = np.cross(force_postion, force) 
    forces_3d = ForceSolver.TranslateReactiveForces3D(forces, pad_leg_connections_3d, base_leg_connections_3d) 
    for i in range(6):
      #translation sum = 0
      #reactive force one pad
      translation_sum = translation_sum + forces_3d[i]
      #moment sum = 0 (taken around 0,0,0 of aboslute frame)
      moment_sum = moment_sum + np.cross(pad_leg_connections_3d[i], forces_3d[i])
      
    results = np.hstack((translation_sum, moment_sum))
    return results
  
  def SolveForces(self, Hexpod : hexpod.Hexpod, position_relative_to_pad, force, max_error = 0.1):
    initial_guess = np.zeros(6)
    result = least_squares(ForceSolver.ForceSolverEquation, initial_guess, args=(
      hexpod.Hexpod.PlatformLegConnectionPos3D(Hexpod.pad_leg_connections, Hexpod.pad_translation, Hexpod.pad_rotation), 
      hexpod.Hexpod.PlatformLegConnectionPos3D(Hexpod.base_leg_connections, Hexpod.base_translation, Hexpod.base_rotation), 
      force, 
      Hexpod.pad_translation + position_relative_to_pad)) 
    
    max_deviation = np.max(np.abs(result.fun))
    if max_deviation > max_error:
        print("Error: Could not solve for pad position and rotation")
        print("Max deviation: ", max_deviation)
        print("Result: ", result.x)
        print("Message: ", result.message)
        print("Status: ", result.status) 
        self.forces = result.x
        return False
    else:
        self.forces = result.x
        return True
      
  def VisualizeForces(self, Hexpod : hexpod.Hexpod, applied_force, applied_force_pos_relative_to_pad, true_scale = False, max_force_size = 1):
    pad_leg_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(Hexpod.pad_leg_connections, Hexpod.pad_translation, Hexpod.pad_rotation)
    
    base_leg_pos = hexpod.Hexpod.PlatformLegConnectionPos3D(Hexpod.base_leg_connections, Hexpod.base_translation, Hexpod.base_rotation)
    
    # 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pad_leg_pos[:, 0], pad_leg_pos[:, 1], pad_leg_pos[:, 2], c='r', marker='o')
    ax.scatter(base_leg_pos[:, 0], base_leg_pos[:, 1], base_leg_pos[:, 2], c='b', marker='o')

    # Connect the legs
    for i in range(6):
        ax.plot([pad_leg_pos[i, 0], base_leg_pos[i, 0]], 
                [pad_leg_pos[i, 1], base_leg_pos[i, 1]], 
                [pad_leg_pos[i, 2], base_leg_pos[i, 2]], c='k')
        
    # Draw the forces
    forces_3d = self.TranslateReactiveForces3D(self.forces, pad_leg_pos, base_leg_pos)
    applied_force_pos_relative_to_abs = Hexpod.pad_translation + applied_force_pos_relative_to_pad
    if not true_scale:
      scaled_forces = (self.forces/ np.max(np.abs(self.forces))) * max_force_size
      applied_force = (applied_force/ np.max(np.abs(self.forces))) * max_force_size
      forces_3d = self.TranslateReactiveForces3D(scaled_forces , pad_leg_pos, base_leg_pos)
    
    # Draw the applied force
    ax.quiver(applied_force_pos_relative_to_abs[0], applied_force_pos_relative_to_abs[1], applied_force_pos_relative_to_abs[2], 
              applied_force[0], applied_force[1], applied_force[2], color='r')
    ax.text(applied_force_pos_relative_to_abs[0], applied_force_pos_relative_to_abs[1], applied_force_pos_relative_to_abs[2], 
          'Applied Force - ' + str(np.round(applied_force,1)), color='black')
    
    for i in range(6):
        ax.quiver(pad_leg_pos[i, 0], pad_leg_pos[i, 1], pad_leg_pos[i, 2], 
                  forces_3d[i, 0], forces_3d[i, 1], forces_3d[i, 2], color='g')
        ax.text(pad_leg_pos[i, 0], pad_leg_pos[i, 1], pad_leg_pos[i, 2], 
          'Force on leg ' + str(i) + ' - ' + str(np.round(self.forces[i],1)), color='black')
    
    # hard to eunderstand the 3d plot without this
    # Set the aspect ratio
    ax.set_aspect('auto')
    limit = np.max(np.abs(np.concatenate((pad_leg_pos, base_leg_pos)))) + 0.1
    ax.set_xlim([-limit, limit])
    ax.set_ylim([-limit, limit])
    ax.set_zlim([-limit, limit])
    fig.set_size_inches(10, 10)
    plt.show()
    return
      
    
    