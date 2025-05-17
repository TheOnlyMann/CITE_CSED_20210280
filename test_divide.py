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
basemesh.load("Calibration cube v3.stl")
basemesh.display("Original mesh")
# check divider
divider = STLDivider("divided.stl", mesh=basemesh.get_copy())
divider.set_z_height(z_height=10.0)
divider.divide()
divider.display()
divider.display("upper", "Upper part")
divider.display("lower", "Lower part")
divider.save("divided_upper.stl","upper")
divider.save("divided_lower.stl","lower")
sliceSTL("divided_upper.stl", outputPath="data/gcode")
sliceSTL("divided_lower.stl", outputPath="data/gcode")
viewGCODE("data/gcode/divided_upper.gcode")
viewGCODE("data/gcode/divided_lower.gcode")

