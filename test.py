from scripts.STLBase import STLBase
from scripts.STLRotater import STLRotater
from scripts.STLUtils import fixSTL, enhanceSTL_via_open3d, evalSTL_initface, evalSTL_base, evalSTL_wall, costcalc, evalSTL_center_of_mass, evalSTL_island, evalSTL, displayeval

import math

basemesh = STLBase("test.stl")
basemesh.load("Calibration cube v3.stl")
basemesh.display("Original mesh")

rotater = STLRotater("rotated.stl", mesh=basemesh.get_copy())
rotater.set_rotation(rotation_angles=[math.radians(90), 0, 0], limit_z_rotation=False)
rotater.apply_rotation()
rotater.display("Rotated mesh")

stl = rotater.transfer()

fixSTL(stl, verbose=True)
stl.display("Fixed Mesh")
enhanceSTL_via_open3d(stl, voxel_factor=200.0, verbose=True)
stl.display("Enhanced Mesh")
evalSTL_initface(stl, override=True, verbose=True)
evalSTL_base(stl, area_threshold=1e-4, tolerance=1e-5, verbose=True)

evalSTL_wall(stl, anglecheck_function=costcalc, verbose=True)

evalSTL_center_of_mass(stl, verbose=True)

island_info = evalSTL_island(stl.get(), layer_height=0.2, verbose=True)


eval_result = evalSTL(stl, area_threshold=1e-4, tolerance=1e-5, layer_height=0.2, verbose=False)
displayeval(stl, eval_result, title="Evaluation Result")