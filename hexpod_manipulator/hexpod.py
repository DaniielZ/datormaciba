import numpy as np
import forward_kinematics

#3d class for a hex pod 
class Hexpod:
  base_translation = np.zeros(3)
  base_rotation = np.zeros((3,3))
  base_leg_connections = np.zeros((6,2))
  
  pad_translation = np.zeros(3)
  pad_rotation = np.zeros((3,3))
  pad_leg_connections = np.zeros((6,2)) 
  
  # 3d array of the 6 legs 
  # 3d pos of base connection, 3d pos of pad connection
  leg_length = np.zeros(6)
  leg_min_length = 0
  leg_max_length = 0
  
  def __init__(self, base_radius, pad_radius, short_vs_long_angle_ratio, leg_min_length : float, leg_max_length : float):
    self.base_leg_connections = self.CalculateHexagon(base_radius, short_vs_long_angle_ratio)
    self.pad_leg_connections = self.CalculateHexagon(pad_radius, short_vs_long_angle_ratio)
    #offset all legsśby one such that leg connections match up
    self.pad_leg_connections = np.roll(self.pad_leg_connections, 1, axis=0)
    
    self.leg_length = leg_min_length*np.ones(6)
    self.leg_min_length = leg_min_length
    self.leg_max_length = leg_max_length
    
    # solve the position and ortof the pad and the base
    forward_kinematics.ForwardKinematics.PadPosRotSolver(self, self.leg_max_length*0.01)
    
    
  
    # everything except the pad postion and rotation is known
  def CalculateHexagon(self, radius, short_vs_long_angle_ratio): 
    hexagon = np.zeros((6,2))
    angles = np.zeros(6)
    short_edge_deg = np.deg2rad(360/6)*short_vs_long_angle_ratio
    long_edge_deg = np.deg2rad(360/6) - short_edge_deg
    
    for i in range(6):
      if i%2 == 0:
          angles[i] = angles[i-1]+short_edge_deg
      else:
          angles[i] = angles[i-1]+long_edge_deg     
      hexagon[i] = [radius*np.cos(angles[i]), radius*np.sin(angles[i])]
    
    return hexagon

  
