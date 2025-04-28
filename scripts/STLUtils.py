import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from import_stl import import_stl
from STLBase import STLBase
import open3d as o3d
from shapely.geometry import Polygon
from shapely.ops import unary_union

def fixSTL(stl: STLBase, verbose: bool = False):
    """
    Fix the given STLBase object using trimesh's built-in geometry fixer.
    This includes repairing inverted faces, normals, degenerate geometry, etc.

    Parameters:
        stl (STLBase): The STLBase object to fix.
        verbose (bool): If True, print fix summary.

    Returns:
        None
    """
    stl._check()
    mesh = stl.get()
    
    if verbose:
        print("🔧 Starting mesh fix...")
        print(f"  - Initial valid state: watertight={mesh.is_watertight}, euler={mesh.euler_number}, volume={mesh.volume:.6f}")

    trimesh.repair.fix_inversion(mesh)
    trimesh.repair.fix_normals(mesh)
    trimesh.repair.fix_winding(mesh)
    trimesh.repair.fill_holes(mesh)


    if verbose:
        print("✅ Mesh fix complete.")
        print(f"  - Final valid state: watertight={mesh.is_watertight}, euler={mesh.euler_number}, volume={mesh.volume:.6f}")
    
    stl.set(mesh)


def enhanceSTL_via_open3d(stl: STLBase, voxel_factor: float = 200.0, verbose: bool = True):
    """
    Enhances STLBase mesh via Open3D vertex clustering remeshing.

    Parameters:
        stl (STLBase): STLBase object to remesh.
        voxel_factor (float): Lower = finer detail. Default=200.
        verbose (bool): Print process info.
    """
    stl._check()
    mesh_tm = stl.get_copy()

    mesh_o3d = o3d.geometry.TriangleMesh()
    mesh_o3d.vertices = o3d.utility.Vector3dVector(mesh_tm.vertices)
    mesh_o3d.triangles = o3d.utility.Vector3iVector(mesh_tm.faces)
    mesh_o3d.compute_vertex_normals()

    # Clean
    mesh_o3d.remove_duplicated_vertices()
    mesh_o3d.remove_degenerate_triangles()
    mesh_o3d.remove_duplicated_triangles()
    mesh_o3d.remove_non_manifold_edges()

    # Remesh
    voxel_size = mesh_o3d.get_max_bound()[0] / voxel_factor
    mesh_o3d = mesh_o3d.simplify_vertex_clustering(
        voxel_size=voxel_size,
        contraction=o3d.geometry.SimplificationContraction.Average
    )
    mesh_o3d.compute_vertex_normals()

    # Convert back to trimesh
    remeshed = trimesh.Trimesh(
        vertices=np.asarray(mesh_o3d.vertices),
        faces=np.asarray(mesh_o3d.triangles),
        process=False
    )
    stl.set(remeshed)
    if verbose:
        print(f"✅ Remeshing complete: {len(mesh_tm.vertices)}->{len(remeshed.vertices)} vertices | {len(mesh_tm.faces)}->{len(remeshed.faces)} faces")


def densifymesh(stl: STLBase, factor: float = 1.0, verbose: bool = False):
    """
    Densifies the mesh by a given factor using Open3D.
    this is used for z axis linear transformation, nonplanar slicing
    """

    raise NotImplementedError("Densify mesh function is not implemented yet.")


def evalSTL_initface(stl: STLBase, override: bool = False, verbose: bool = False):
    """
    force a certain datatype for stl.face_tag
    this will be a (face type , cost value) tuple for each face in the mesh
    :param stl: STLBase object
    :return: None
    """
    if stl.face_tag is not None and not override:
        print("Face tag already set.")
        return
    stl._check()
    mesh = stl.get()
    facelen = len(mesh.faces)
    
    stl.face_tag = np.zeros(facelen, dtype=[('face_type', 'U10'), ('cost', 'f4')])
    stl.face_tag['face_type'] = 'unknown'
    stl.face_tag['cost'] = 0.0

    if verbose:
        print("✅ Face tags initialized.")
        print(f"  - Number of faces: {facelen}")
        print(f"override: {override}")


def evalSTL_base(stl:STLBase, area_threshold=1e-4, tolerance=1e-5,base_cost_factor = -1, verbose: bool = False):
    """
    Check the base of the STLBase object and return the area of the base mesh.
    A face is considered part of the base if all its vertices are at the bottom Z level.

    Parameters:
        stl (STLBase): The STLBase object to check.

    Returns:
        float: Area of the base mesh if valid, otherwise -1
        np.ndarray: Indices of base faces
    """
    stl._check()
    mesh = stl.get_copy()

    # Find the lowest Z value in the mesh
    z_min = mesh.bounds[0][2]
    base_faces = []
    
    # Collect all face indices whose vertices are at z_min level
    for i, face in enumerate(mesh.faces):
        z_coords = mesh.vertices[face][:, 2]
        if np.all(np.abs(z_coords - z_min) < tolerance):
            stl.face_tag[i]['face_type'] = 'base'
            stl.face_tag[i]['cost'] = base_cost_factor
            base_faces.append(i)
    
    base_area = np.sum(mesh.area_faces[base_faces]) if base_faces else 0.0

    if base_area < area_threshold:
        if verbose:
            print("❌ Base area too small.")
        return -1.0
    if verbose:
        print(f"✅ Base area valid: {base_area:.6f}")
    return base_area,base_faces



def costcalc(normal: np.ndarray, z_axis: np.ndarray, tolerance: float = 1e-5, inner_angle_tolerance: tuple = (55.0, 70.0), outer_angle_tolerance: tuple = (35.0, 70.0)):
    """
    Calculate the angle between a normal vector and the Z-axis, and return a penalty based on the angle.
    Penalty is done in a ReLU manner however both for the upper extreme and lower extreme, both cases being close to vertical/horizontal being less responsive than being diagonal.
    """
    angle = np.arccos(np.clip(np.dot(normal, z_axis), -1.0, 1.0))
    angle = np.degrees(angle)
    #check if the face is pointing directly the top, if so return 0 penalty(unlike bottom facing faces)
    if angle < tolerance:
        return 0.0
    tolerance_val = inner_angle_tolerance if angle < 90.0 else outer_angle_tolerance
    deviation = abs(angle - 90.0)

    penalty = 0.0
    if deviation < tolerance_val[0]:
        penalty = 0.0
    elif deviation < tolerance_val[1]:
        penalty = (deviation - tolerance_val[0]) / (tolerance_val[1] - tolerance_val[0])
    else:
        penalty = 1.0
    return penalty
    
    

def evalSTL_wall(stl: STLBase,anglecheck_function = costcalc, verbose: bool = False):
    """
    Check the wall of the STLBase object and return the wall area and penalty.
    both inner and outer angles are checked, if the angle is outside the tolerance penalty gets harsher
    within the angles it still prefers to be vertical, however the penalty is less harsh
    
    Parameters:
        stl (STLBase): The STLBase object to check.
        anglecheck_function: function: Function to check angles between faces.
        tolerance (float): Numerical tolerance for angle checks.

    Returns:
        tuple: Wall area and penalty value.
    """
    stl._check()
    mesh = stl.get()
    z_axis = np.array([0, 0, 1])

    total_area = 0.0
    total_penalty = 0.0

    for i, (normal, area) in enumerate(zip(mesh.face_normals, mesh.area_faces)):
        if stl.face_tag[i]['face_type'] == 'base':
            continue
        penalty = anglecheck_function(normal, z_axis)
        stl.face_tag[i]['face_type'] = 'wall'
        stl.face_tag[i]['cost'] = penalty
        total_area += area
        total_penalty += area * penalty

    if verbose:
        print(f"✅ Wall check complete. Area: {total_area:.4f}, Penalty: {total_penalty:.4f}")
    return total_area, total_penalty

def evalSTL_center_of_mass(stl: STLBase, verbose: bool = False):
    """
    Evaluate the center of mass of the STLBase object.

    Parameters:
        stl (STLBase): The STLBase object to evaluate.

    Returns:
        np.ndarray: Center of mass coordinates.
    """
    stl._check()
    mesh = stl.get()
    com = mesh.center_mass
    if verbose:
        print(f"✅ Center of mass: {com}")
    return com

def paths_to_polygons(paths, area_threshold=1e-6):
    polygons = []
    for entity in paths.entities:
        try:
            points = paths.vertices[entity.points]
            if len(points) >= 3:
                poly = Polygon(points)
                if poly.is_valid and poly.area > area_threshold:
                    polygons.append(poly)
        except Exception:
            continue
    return polygons

def evalSTL_island(mesh: trimesh.Trimesh, layer_height: float = 0.2, verbose: bool = False):
    """
    Detect unsupported island polygons at each layer based on overlap with previous layer.
    Returns a list of dictionaries: {'z': height, 'islands': [Polygon, ...]}.

    Parameters:
        mesh (trimesh.Trimesh): The mesh to analyze.
        layer_height (float): Z interval per slice.
        verbose (bool): Print summary if True.

    Returns:
        List[Dict]: List of {z, islands}, each containing unsupported island polygons per layer.
    """
    z_min, z_max = mesh.bounds[0][2], mesh.bounds[1][2]
    n_layers = int((z_max - z_min) / layer_height) + 1

    prev_polys = []
    island_info = []

    for layer in range(n_layers):
        z = z_min + layer * layer_height
        section = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if section is None:
            continue

        try:
            paths = section.to_2D()[0]
        except Exception:
            continue

        polys = paths_to_polygons(paths)

        if layer == 0:
            prev_polys = polys
            continue

        unsupported = []
        for poly in polys:
            if not any(poly.intersects(p) for p in prev_polys):
                unsupported.append(poly)

        if unsupported:
            island_info.append({
                "z": z,
                "islands": unsupported
            })

        prev_polys = polys

    if verbose:
        print(f"📏 Detected islands in {len(island_info)} layers.")
    return island_info


def evalSTL(stl: STLBase, area_threshold=1e-4, tolerance=1e-5, layer_height=0.2, verbose: bool = False):
    """
    Evaluate the STLBase object for various properties.

    Parameters:
        stl (STLBase): The STLBase object to evaluate.
        area_threshold (float): Minimum area for base check.
        tolerance (float): Tolerance for Z-level check.
        layer_height (float): Z interval per slice.
        verbose (bool): Print summary if True.

    Returns:
        None
    """
    evalSTL_initface(stl, override=True, verbose=verbose)
    base_area, base_faces = evalSTL_base(stl, area_threshold=area_threshold, tolerance=tolerance, verbose=verbose)
    wall_area, penalty_value = evalSTL_wall(stl, anglecheck_function=costcalc, verbose=verbose)
    center_of_mass = evalSTL_center_of_mass(stl, verbose=verbose)
    island_info = evalSTL_island(stl.get(), layer_height=layer_height, verbose=verbose)

    #cost calculation
    cost = 0.0
    cost = np.sum(stl.face_tag['cost'] * stl.mesh.area_faces)
    print (f"Total cost: {cost:.4f}")

def displayeval(stl: STLBase, title: str = ""):
    """
    Displays the original mesh in 3D.
    The mesh is displayed with a cyan color and a black edge.
    If the mesh is not set, it will print it out and return
    """
    mesh = stl.get_copy()
    if title == "":
        title = stl.filename
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    mesh_faces = mesh.vertices[mesh.faces]
    ax.add_collection3d(Poly3DCollection(mesh_faces, facecolors='cyan', linewidths=0.5, edgecolors='k', alpha=.25))
    ax.set_box_aspect(aspect=[1, 1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    plt.show()


if __name__ == "__main__":
    # Example usage
    stl = STLBase("example.stl")
    stl.load("Calibration cube v3.stl")
    stl.display("Original Mesh")
    fixSTL(stl, verbose=True)
    stl.display("Fixed Mesh")
    enhanceSTL_via_open3d(stl, voxel_factor=200.0, verbose=True)
    stl.display("Enhanced Mesh")
    evalSTL_initface(stl, override=True, verbose=True)
    evalSTL_base(stl, area_threshold=1e-4, tolerance=1e-5, verbose=True)

    evalSTL_wall(stl, anglecheck_function=costcalc, verbose=True)

    evalSTL_center_of_mass(stl, verbose=True)

    island_info = evalSTL_island(stl.get(), layer_height=0.2, verbose=True)
    for info in island_info:
        print(f"Layer Z: {info['z']}, Islands: {len(info['islands'])}")
        for island in info['islands']:
            print(f"  Island Area: {island.area:.4f}")

    evalSTL(stl, area_threshold=1e-4, tolerance=1e-5, layer_height=0.2, verbose=False)