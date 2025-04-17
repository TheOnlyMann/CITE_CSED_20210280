from sliced_datatype import MeshSample
import trimesh
import numpy as np

def check_mesh(tri_mesh: trimesh.Trimesh) -> bool:
    """
    Check the base mesh and return if it is valid.
    :param datatype: SlicedDataType object
    :return: True if valid, False otherwise
    """
    if tri_mesh is None:
        print("Mesh not set.")
        return False
    if not tri_mesh.is_watertight:
        print("Mesh is not watertight.")
        return False
    if not tri_mesh.is_convex:
        print("Mesh is not convex.")
        return False
    if len(tri_mesh.faces) == 0:
        print("Mesh has no faces.")
        return False
    if len(tri_mesh.vertices) == 0:
        print("Mesh has no vertices.")
        return False
    return True

def check_base(datatype: MeshSample, area_threshold=1e-4, tolerance=1e-5):
    """
    Check the base of the [rotated mesh] and return the area of the base mesh.
    A face is considered part of the base if all its vertices are at the bottom Z level.

    :param datatype: MeshSample object
    :param area_threshold: Minimum total base area required
    :param tolerance: Numerical tolerance for Z-level check
    :return: Area of the base mesh if valid, otherwise -1
    """
    if datatype.base_mesh is None:
        print("Base mesh not set.")
        return -1

    # Use rotated mesh if available
    mesh = datatype.rotated_mesh.copy() if datatype.rotated_mesh else datatype.base_mesh.copy()

    # Find the lowest Z value in the mesh
    z_min = mesh.bounds[0][2]

    # Collect all face indices whose vertices are at z_min level
    base_faces = []
    for i, face in enumerate(mesh.faces):
        z_coords = mesh.vertices[face][:, 2]
        if np.all(np.abs(z_coords - z_min) < tolerance):
            base_faces.append(i)


    if not base_faces:
        print("❌ No faces lie flat on the XY plane.")
        return -1

    # Sum the area of those base faces
    datatype.base_faces = base_faces
    base_area = np.sum(mesh.area_faces[base_faces])
    if base_area < area_threshold:
        print(f"❌ Base area too small: {base_area:.6f}")
        return -1

    print(f"✅ Base is valid. Area: {base_area:.6f}")
    return base_area


def check_wall(datatype: MeshSample, wall_threshold=60, penalty_power=2, tolerance=1e-5):
    """
    Evaluate wall normals of the mesh against the build platform.

    Downward-facing overhangs are penalized heavily.
    Normals perpendicular to the floor (vertical walls) are best.
    Upward-facing walls (inward slope) are acceptable with small penalty.
    excludes the floor mesh from datatype.base_faces

    Parameters:
        datatype (MeshSample): Object holding the rotated mesh.
        wall_threshold (float): Maximum angle from vertical (degrees) that is freely allowed.
        penalty_power (int): Controls how harsh the overhang penalty is (quadratic/cubic etc).

    Returns:
        float: Cumulative penalty score (lower is better). Can be thresholded externally.
    """
    mesh = datatype.get_mesh()
    if check_mesh(mesh) == False:
        return float("inf")
    # Calculate the angle between the face normals and the Z-axis
    face_normals = mesh.face_normals
    z_axis = np.array([0, 0, 1])


