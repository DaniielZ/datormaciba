import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

#3d class for a hex pod 
class Hexpod:
  base_translation = np.zeros(3)
  base_rotation = np.zeros(3) #euler angles in radians
  base_leg_connections = np.zeros((6,2))
  
  pad_translation = np.zeros(3)
  pad_rotation = np.zeros(3) #euler angles in radians
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
    return
  
    # everything except the pad postion and rotation is known
  def CalculateHexagon(self, radius, short_vs_long_angle_ratio): 
    hexagon = np.zeros((6,2))
    angles = np.zeros(6)
    short_edge_deg = (360/3)*short_vs_long_angle_ratio
    long_edge_deg = (360/3) - short_edge_deg
    
    for i in range(6):
      if i%2 == 0:
          angles[i] = angles[i-1]+short_edge_deg
      else:
          angles[i] = angles[i-1]+long_edge_deg     
      hexagon[i] = [radius*np.cos(np.deg2rad(angles[i])), radius*np.sin(np.deg2rad(angles[i]))]
    
    return hexagon
  
  def PrintStatus(self):
    print("Base leg connections: ", self.base_leg_connections)
    print("Pad leg connections: ", self.pad_leg_connections)
    print("Leg length: ", self.leg_length)
    print("Pad translation: ", self.pad_translation)
    print("Pad rotation: ", self.pad_rotation)
    print("Base translation: ", self.base_translation)
    print("Base rotation: ", self.base_rotation)
    return
  
  
  # input is trannslation and rotation of the pad rad ians
  @staticmethod
  def PlatformLegConnectionPos3D(leg_connections_2d, translation, rotation):
    # Apply rotations
    rotation_matrix = R.from_euler('xyz', rotation).as_matrix()
    # Make a 3D array of the 2D leg connections
    leg_connections_3d = np.hstack((leg_connections_2d, np.zeros((6,1))))
    # Apply translations and rotations
    
    for i in range(6):
      leg_connections_3d[i] = (leg_connections_3d[i] @ rotation_matrix.T) + translation
    return leg_connections_3d
    
    
  def VisualizeAllLegConnectionPositions(self):
    # Apply rotations

    # Apply translations and rotations
    pad_leg_pos = Hexpod.PlatformLegConnectionPos3D(self.pad_leg_connections, self.pad_translation, self.pad_rotation)
    base_leg_pos = Hexpod.PlatformLegConnectionPos3D(self.base_leg_connections, self.base_translation, self.base_rotation)
    
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
  
  def VisualizeHexagon(self, hexagon):
    fig, ax = plt.subplots()
    ax.scatter(hexagon[:,0], hexagon[:,1])
    #label
    for i in range(6):
        ax.annotate(str(i), (hexagon[i,0], hexagon[i,1]))
    plt.show()
    return
  
# 1 is max, 0 is min
  def SetLegLengthFromRatio(self, ratio_array):
    ratio_array = np.array(ratio_array)  # Convert to NumPy array
    self.leg_length = self.leg_min_length + ratio_array * (self.leg_max_length - self.leg_min_length)
    return
  
