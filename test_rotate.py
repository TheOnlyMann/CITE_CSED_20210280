from scripts.STLBase import STLBase
from scripts.STLRotater import STLRotater
from scripts.STLDivider import STLDivider
from scripts.STLExtender import STLExtender
from scripts.STLWarper import STLWarper, warpmethod_1, read_and_warp_gcode
from scripts.STLUtils import fixSTL, enhanceSTL_via_open3d, evalSTL_initface, evalSTL_base, evalSTL_wall, costcalc, evalSTL_center_of_mass, evalSTL_island, evalSTL, displayeval
from scripts.PointCloudUtils import extractPointCloud, display_point_cloud
from scripts.prusa_slicer import sliceSTL, repairSTL, viewGCODE
import math
import numpy as np
import open3d as o3d

# Load the original mesh
basemesh = STLBase("test.stl")
basemesh.load("test_shape_2.stl")
basemesh.display("Original mesh")
displayeval(basemesh, evalSTL(basemesh))
basemesh.save("rotated_before.stl")
sliceSTL("rotated_before.stl", outputPath="data/gcode")
# check rotater
rotator = STLRotater("rotated.stl", mesh=basemesh.get_copy())
rotator.set_rotation(rotation_angles=[math.radians(90), math.radians(0), 0], limit_z_rotation=True)
rotator.apply_rotation()
rotator.display("Rotated mesh")
displayeval(rotator.outputfile, evalSTL(rotator.outputfile))

rotator.save("rotated.stl")
sliceSTL("rotated.stl", outputPath="data/gcode")
viewGCODE("data/gcode/rotated.gcode")

