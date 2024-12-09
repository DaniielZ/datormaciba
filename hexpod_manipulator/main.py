import numpy as np
from hexpod import Hexpod
from forward_kinematics import ForwardKinematics
from force_solver import ForceSolver
from inverse_kinematics import InverseKinematics
#https://loguru.readthedocs.io/en/stable/overview.html


#Hexpod defintion for forward kinematics solving
hexpod = Hexpod(0.4, 0.2, 0.3, 0.5, 2)

hexpod.pad_rotation = np.array([0,0,np.deg2rad(30)])  
hexpod.pad_translation = np.array([0,0,1])
applied_force = np.array([0, 0, -9.81 * 0.5])
applied_force_pos_relative_to_pad = np.array([0, 1, 0])

fk = ForwardKinematics()
fs = ForceSolver() 
ik = InverseKinematics()

#Solve for pad position and rotation

# Test case 1
hexpod.SetLegLengthFromRatio([0.3,0.3,0.3,0.3,0.3,0.3])
fk.PadPosRotSolver(hexpod, 0.01) 
hexpod.VisualizeAllLegConnectionPositions() #

# Test case 2
# hexpod.SetLegLengthFromRatio([0.9,0.9,1,1,1,1])
# fk.PadPosRotSolver(hexpod, 0.01) 
# hexpod.VisualizeAllLegConnectionPositions() 

# Test case 3
# hexpod.SetLegLengthFromRatio([0.9,0.9,1,1,0.8,0.8])
# fk.PadPosRotSolver(hexpod, 0.01) 
# hexpod.VisualizeAllLegConnectionPositions() 

fs.SolveForces(hexpod, applied_force_pos_relative_to_pad, applied_force, max_error = 0.01)
# print(fs.forces)


fs.VisualizeForces(hexpod, true_scale=False, max_force_size=0.5, applied_force=applied_force,applied_force_pos_relative_to_pad=applied_force_pos_relative_to_pad)


# IK Test case 1
hexpod.pad_translation = np.array([0,0,1])
hexpod.pad_rotation = np.array([np.deg2rad(30),np.deg2rad(30),np.deg2rad(30)])
ik.LegLeangthSolver(hexpod, 0.01)
hexpod.VisualizeAllLegConnectionPositions()

#TODO
# PDFinņš Dairim
