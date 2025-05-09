from scripts.STLBase import STLBase
from scripts.STLRotater import STLRotater
from scripts.STLUtils import fixSTL, enhanceSTL_via_open3d, evalSTL_initface, evalSTL_base, evalSTL_wall, costcalc, evalSTL_center_of_mass, evalSTL_island, evalSTL, displayeval
from scripts.PointCloudUtils import extractPointCloud, display_point_cloud
from scripts.STLDivider import STLDivider
import math
import numpy as np
import open3d as o3d

basemesh = STLBase("test.stl")
basemesh.load("Calibration cube v3.stl")
basemesh.display("Original mesh")

rotater = STLRotater("rotated.stl", mesh=basemesh.get_copy())
rotater.set_rotation(rotation_angles=[math.radians(90), math.radians(90), 0], limit_z_rotation=False)
rotater.apply_rotation()
rotater.display("Rotated mesh")

divider = STLDivider("divided.stl", mesh=rotater.get_copy())
print(f"Z limit: {divider.get_z_limit()}")
divider.set_z_height(z_height=-30.0)
divider.divide()
divider.display()
divider.display("upper")
divider.display("lower")

stl = rotater.transfer()

fixSTL(stl, verbose=True)
#stl.display("Fixed Mesh")

enhanceSTL_via_open3d(stl, voxel_factor=2.0, verbose=True)
stl.display("Enhanced Mesh")
evalSTL_initface(stl, override=True, verbose=True)
evalSTL_base(stl, area_threshold=1e-4, tolerance=1e-5, verbose=True)

evalSTL_wall(stl, anglecheck_function=costcalc, verbose=True)

evalSTL_center_of_mass(stl, verbose=True)

island_info = evalSTL_island(stl.get(), layer_height=0.2, verbose=True)


eval_result = evalSTL(stl, area_threshold=1e-4, tolerance=1e-5, layer_height=0.2, verbose=False)
displayeval(stl, eval_result, title="Evaluation Result")


point_cloud = extractPointCloud(stl, point_num=2048, normalize=True,visualize=True, verbose=True)
print(f"Point cloud shape: {point_cloud.shape}")
print(f"Point cloud mean: {np.mean(point_cloud, axis=0)}")
print(f"Point cloud max norm: {np.max(np.linalg.norm(point_cloud, axis=1))}")
pcr = o3d.geometry.PointCloud()
pcr.points = o3d.utility.Vector3dVector(point_cloud)

#display_point_cloud(pcr, window_name="Point Cloud")