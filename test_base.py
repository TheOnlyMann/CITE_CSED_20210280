from scripts.STLBase import STLBase
from scripts.STLRotater import STLRotater
from scripts.STLDivider import STLDivider
from scripts.STLExtender import STLExtender
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
basemesh.save("base.stl")

sliceSTL("base.stl", outputPath="data/gcode")
viewGCODE("data/gcode/base.gcode")