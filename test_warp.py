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


def warpmethod2(x,y):
    return np.sqrt(x**2 + y**2) * 0.2
def warpmethod3(x,y):
    return (np.abs(x)+np.abs(y)) * 0.2

# Load the original mesh
basemesh = STLBase("test.stl")
basemesh.load("best_mesh.stl")
basemesh.display("Original mesh")
basemesh.save("warped_before.stl")
sliceSTL("warped_before.stl", outputPath="data/gcode")
# check warper
warper = STLWarper("warped.stl", mesh=basemesh.get_copy())
warper.set_warp_method(warpmethod_1)


warper.preprocess()
warper.warp()
warper.display("Warped mesh")
warper.slice_and_reverse()

viewGCODE("data/gcode/warped.gcode")