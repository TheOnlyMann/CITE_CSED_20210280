from sliced_datatype import SlicedDataType
import trimesh
import numpy as np

def slice_mesh(mesh: trimesh.Trimesh, z_height: float) -> trimesh.Trimesh:
    """
    Slice the mesh at a given Z height and return the sliced mesh.
    :param mesh: Trimesh object to be sliced
    :param z_height: Z height at which to slice the mesh
    :return: custom tuple containing the sliced mesh and the plane used for slicing, as well as the "wall" info of the slice(it's the degree difference between the normal of the plane and the normal of the mesh face)
    """
    # Create a plane at the specified Z height
    plane_normal = [0, 0, 1]  # Normal vector for the XY plane
    plane_origin = [0, 0, z_height]  # Point on the plane
    plane_transform = trimesh.transformations.translation_matrix(plane_origin)
    plane_transform[:3, :3] = trimesh.transformations.rotation_matrix(0, plane_normal)[:3, :3]
    plane = trimesh.path.creation.plane(plane_transform, width=mesh.extents[0], height=mesh.extents[1])

    # Slice the mesh using the plane
    sliced_mesh = mesh.slice_plane(plane)
    # Calculate the angle between the plane normal and the mesh face normals
    face_normals = mesh.face_normals
    plane_normal = np.array(plane_normal)
    wall_info = np.degrees(np.arccos(np.clip(np.dot(face_normals, plane_normal), -1.0, 1.0)))