import numpy as np
import hexpod
import forward_kinematics



#Hexpod defintion for forward kinematics solving
hexpod = hexpod.Hexpod(0.4, 0.2, 0.3, 1, 2)
hexpod.pad_rotation = np.array([0,0,np.deg2rad(30)])  
hexpod.pad_translation = np.array([0,0,1])

# hexpod.PrintStatus()
# hexpod.VisualizeHexagon(hexpod.base_leg_connections)
# hexpod.VisualizeHexagon(hexpod.pad_leg_connections)
# hexpod.VisualizeAllLegConnectionPositions()

#Forward kinematics solver
fk = forward_kinematics.ForwardKinematics()

#Solve for pad position and rotation

fk.PadPosRotSolver(hexpod, 0.01)
hexpod.VisualizeAllLegConnectionPositions()
hexpod.SetLegLengthFromRatio([0.3,0.3,0.5,0.5,0.6,0.6])
fk.PadPosRotSolver(hexpod, 0.01)
hexpod.VisualizeAllLegConnectionPositions()
hexpod.SetLegLengthFromRatio([1,1,1,1,1,1])


#TODO
# add force solver
# add inverse kinematics solver
# add gui for setting leg lengths or roations
