from scripts.import_stl import import_stl
from scripts.sliced_datatype import MeshSample
import scripts.prusa_slicer as ps

import trimesh
import matplotlib.pyplot as plt
import numpy as np
from trimesh.transformations import rotation_matrix

test_stl = import_stl("Calibration cube v3")
print(test_stl.metadata)
#mesh_plot_info(test_stl)

datatype = MeshSample(test_stl, "Calibration cube v3")
datatype.display_basemesh()


datatype.set_rotation(angles_rad = [1, 0, 3])
datatype.apply_rotation()
datatype.display_rotation(scale = 0.03)

datatype.apply_rotation()
#datatype.slice_mesh(0.2)
#datatype.display_transformed()

#temp save STL
datatype.save_stl("test_rotation.stl")
ps.sliceSTL("test_rotation.stl")
ps.viewGCODE("output.gcode")

