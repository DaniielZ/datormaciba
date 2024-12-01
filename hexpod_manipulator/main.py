import numpy as np
import hexpod




#Hexpod defintion for forward kinematics solving
hexpod = hexpod.Hexpod(0.4, 0.2, 0.3, 1, 2)
print("Base leg connections: ", hexpod.base_leg_connections)
print("Pad leg connections: ", hexpod.pad_leg_connections)
print("Leg length: ", hexpod.leg_length)
print("Pad translation: ", hexpod.pad_translation)
print("Pad rotation: ", hexpod.pad_rotation)
print("Base translation: ", hexpod.base_translation)
print("Base rotation: ", hexpod.base_rotation)
