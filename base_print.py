from scripts.STLBase import STLBase
from scripts.STLRotater import STLRotater
from scripts.STLUtils import fixSTL, enhanceSTL_via_open3d, evalSTL_initface, evalSTL_base, evalSTL_wall, costcalc, evalSTL_center_of_mass, evalSTL_island, evalSTL, displayeval
from scripts.PointCloudUtils import extractPointCloud, display_point_cloud
from scripts.STLDivider import STLDivider
from scripts.prusa_slicer import sliceSTL, repairSTL, viewGCODE
import math
import numpy as np
import open3d as o3d

basemesh = STLBase("calicube3_base.stl")
basemesh.load("Calibration cube v3.stl")
basemesh.display("Original mesh")
basemesh.save("calicube3_base.stl")

sliceSTL("data/stl/calicube3_base.stl", outputPath = "data/gcode")