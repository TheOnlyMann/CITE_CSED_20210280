import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scripts.import_stl import import_stl
from scripts.STLBase import STLBase
import open3d as o3d
from shapely.geometry import Polygon
from shapely.ops import unary_union
import matplotlib.cm as cm

def extractPointCloud(stl: STLBase, point_num: int = 1024, normalize: bool = True, visualize:bool = False,verbose: bool = False):
    """
    Extracts a uniformly sampled point cloud from the STLBase mesh using Open3D.

    Parameters:
        stl (STLBase): The STLBase object to extract from.
        point_num (int): Number of points to sample.
        normalize (bool): If True, normalize to unit sphere.
        verbose (bool): Print point cloud info if True.

    Returns:
        np.ndarray: (N, 3) point cloud array.
    """
    stl._check()
    mesh = stl.get_copy()

    # Convert to Open3D mesh
    o3d_mesh = o3d.geometry.TriangleMesh()
    o3d_mesh.vertices = o3d.utility.Vector3dVector(mesh.vertices)
    o3d_mesh.triangles = o3d.utility.Vector3iVector(mesh.faces)
    o3d_mesh.compute_vertex_normals()

    # Sample point cloud
    pcd = o3d_mesh.sample_points_poisson_disk(number_of_points=point_num)
    if visualize:
        o3d.visualization.draw_geometries([pcd], window_name="Sampled Point Cloud")
    # Normalize
    points = np.asarray(pcd.points)
    if normalize:
        points = points - np.mean(points, axis=0)
        scale = np.max(np.linalg.norm(points, axis=1))
        points = points / scale

    if verbose:
        print(f"✅ Sampled {len(points)} points. Mean: {np.mean(points, axis=0)}, Max norm: {np.max(np.linalg.norm(points, axis=1))}")

    return points  # For PointNet input (shape: [N, 3])

def display_point_cloud(pcd: o3d.geometry.PointCloud, window_name="Point Cloud"):
    """
    Displays a point cloud using Open3D visualizer.

    Parameters:
        pcd (o3d.geometry.PointCloud): The point cloud to display.
        window_name (str): Window title for visualization.
    """
    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError("Input must be an Open3D PointCloud.")
    
    o3d.visualization.draw_geometries([pcd], window_name=window_name)


