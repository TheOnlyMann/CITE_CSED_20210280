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
basemesh.load("test_shape.stl")
displayeval(basemesh, evalSTL(basemesh))
basemesh.save("best_mesh_before.stl")
sliceSTL("best_mesh_before.stl", outputPath="data/gcode")

#use rotater for finding best position
rotater = STLRotater("rotated_best.stl", mesh=basemesh.get_copy())
max_list = len(rotater.inputfile.mesh.faces)
list_len = 10 if max_list > 10 else max_list
#list_len = max_list
#find list_len amount of faces that have the most area
# 각 face의 면적 계산
face_areas = rotater.inputfile.mesh.area_faces

# 면적 기준으로 인덱스 정렬 (내림차순)
sorted_indices = np.argsort(face_areas)[::-1]

# 가장 큰 face N개 인덱스
top_face_indices = sorted_indices[:list_len]

# add random ones to the list
for i in range(min(max_list - 10, 20)):
    top_face_indices = np.append(top_face_indices, np.random.randint(0, max_list))


print(f"Top {list_len} faces with largest areas:")
for i, idx in enumerate(top_face_indices):
    print(f"{i+1}. Face #{idx}, Area: {face_areas[idx]:.6f}")

best_cost = 1e10
best_index = 0
best_mesh = STLBase("best_mesh.stl")
#find the best rotation
for i in range(len(top_face_indices)):
    rotater.set_rotation(bottom_selection=top_face_indices[i],limit_z_rotation=True)
    rotater.apply_rotation()
    #rotater.display(f"Rotated mesh {i+1}")
    base_area, base_faces, wall_area, penalty_value, center_of_mass, island_info, cost = evalSTL(rotater.outputfile)
    tot_cost = -base_area * 20 + penalty_value + center_of_mass[2] + len(island_info) * 100#total cost
    if tot_cost < best_cost:
        best_cost = tot_cost
        best_index = i
        best_mesh.set(rotater.get_copy())
return_stuff = evalSTL(best_mesh)
displayeval(best_mesh, return_stuff)
print(f"Best rotation is {best_index+1} with cost {best_cost}")
best_mesh.save("best_mesh.stl")
#turn it into gcode
sliceSTL("best_mesh.stl", outputPath="data/gcode")
viewGCODE("data/gcode/best_mesh.gcode")
