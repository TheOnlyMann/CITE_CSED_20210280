#python function to convert gcode Z height to inverse Z height
#Wewill only move the z coordinates accordingly and we will not scale down/up the z scales for "proper" non-planar slicing

from sliced_datatype import MeshSample
import trimesh
import numpy as np
import prusa_slicer as ps

#function that enhances given stl mesh's vertexes to be more "plentiful" for the vertex movement

def uniform_surface_densify(mesh: trimesh.Trimesh, iterations=1):
    """
    Uniformly subdivide each triangle face of the mesh.

    Parameters:
        mesh (trimesh.Trimesh): Input mesh to densify.
        iterations (int): Number of times to recursively subdivide each face.
                          Each iteration splits every triangle into 4 smaller ones.

    Returns:
        trimesh.Trimesh: Densified mesh with more uniformly distributed triangles.
    """
    dense = mesh.copy()
    for _ in range(iterations):
        dense = dense.subdivide()
    return dense

def mesh_fix(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """
    Fix the mesh by removing duplicate vertices and filling holes.

    Parameters:
        mesh (trimesh.Trimesh): The input mesh.

    Returns:
        trimesh.Trimesh: A fixed version of the mesh.
    """
    # Remove duplicate faces and vertices
    mesh.remove_duplicate_faces()

    # Fill holes in the mesh
    if not mesh.is_watertight:
        mesh.fill_holes()

    return mesh

def parabola_function(x,y,z):
    """
    Example Z function that creates a parabolic surface.
    :param x: X coordinate
    :param y: Y coordinate
    :param z: Original Z coordinate (not used in this example)
    :return: New Z coordinate based on a parabolic function
    """
    return 0.001 * (x**2 + y**2) * z # Example parabolic function

def transform_stl(mesh: trimesh.Trimesh, z_function):
    """
    Transform the Z coordinates of the mesh vertices using a given function.

    Parameters:
        mesh (trimesh.Trimesh): The input mesh.
        z_function (callable): A function that takes (x, y) coordinates and returns a new Z value.

    Returns:
        trimesh.Trimesh: The transformed mesh with updated Z coordinates.
    """
    # Get the vertices of the mesh
    vertices = mesh.vertices.copy()

    # Apply the transformation to the Z coordinates
    for i in range(len(vertices)):
        x, y,z = vertices[i][0], vertices[i][1], vertices[i][2]
        vertices[i][2] = z_function(x, y,z)

    # Update the mesh with the new vertices
    mesh.vertices = vertices

    return mesh

if __name__ == "__main__":
    # Example usage
    mesh = trimesh.load("test.stl")  # Load your STL file here

    # Densify the mesh surface
    densified_mesh = uniform_surface_densify(mesh, iterations=3)

    #show the new mesh

    densified_mesh.show()

    modified_mesh = transform_stl(densified_mesh, parabola_function)
    
    #show the new mesh
    modified_mesh.show()

    # Fix the mesh
    fixed_mesh = mesh_fix(densified_mesh)

    # Save or display the fixed mesh
    fixed_mesh.export("fixed_example.stl")

    ps.sliceSTL("fixed_example.stl") # Add your parameters here

    ps.viewGCODE("output.gcode")  # Open the GCode file in Prusa GCode Viewer