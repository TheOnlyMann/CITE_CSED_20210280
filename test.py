from scripts.import_stl import import_stl
from scripts.import_stl import mesh_plot_info
from scripts.sliced_datatype import MeshSample

import trimesh
import matplotlib.pyplot as plt
import numpy as np
from trimesh.transformations import rotation_matrix

test_stl = import_stl("Calibration cube v3")
print(test_stl.metadata)
#mesh_plot_info(test_stl)

datatype = MeshSample(test_stl, "Calibration cube v3")
datatype.display_basemesh()

angle_deg = 45
angle_rad = np.radians(angle_deg)
axis = [0, 1, 0]  # Y-axis
origin = [0, 0, 0]  # rotate around origin

R = rotation_matrix(angle_rad, axis, origin)

datatype.set_rotation(R)
datatype.display_rotation(scale = 0.03)

datatype.apply_rotation()
datatype.slice_mesh(0.2)
datatype.display_transformed()