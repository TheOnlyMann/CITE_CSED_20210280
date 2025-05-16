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
# check extender
extender = STLExtender("extender.stl", mesh=basemesh.get_copy())
extender.set_disc_state(disc_radius=5, disc_height=0.2, disc_threshold=90.0, disc_area_threshold=1e-4)
extender.find_bottom_selection(tolerance=1e-5, angle_threshold=92.0, area_threshold=1e-4, verbose=True)
extender.add_disc()
extender.display("Bottom face selection")

#turn it into gcode
extender.save("extender.stl")

sliceSTL("extender.stl")
viewGCODE("data\gcode\extender.gcode")