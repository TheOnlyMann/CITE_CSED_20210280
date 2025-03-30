import trimesh
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D

class MeshSample:
    def __init__(self, original_mesh: trimesh.Trimesh, name: str = "unnamed"):
        self.name = name
        self.original_mesh = original_mesh
        self.rotated_mesh = None
        self.transformed_mesh = None
        self.sliced_mesh = None
        self.rotation_matrix = None
        self.vertex_function = None# function to be applied to vertices
        self.vertex_inverse_function = None# function to be applied to vertices to get back to original mesh
        self.point_cloud = None
        self.slices = []  # list of sliced sections
        self.evaluation = {}  # stores scores like inclination, stability, etc.

    def display_basemesh(self):
        mesh = self.original_mesh
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        mesh_faces = mesh.vertices[mesh.faces]
        ax.add_collection3d(Poly3DCollection(mesh_faces, facecolors='cyan', linewidths=0.5, edgecolors='k', alpha=.25))
        ax.set_box_aspect(aspect = [1,1,1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(self.name)
        plt.show()

    def get_mesh(self):
        return self.transformed_mesh if self.transformed_mesh is not None else self.rotated_mesh if self.rotated_mesh is not None else self.original_mesh
    
    def set_rotation(self, rotation_matrix = np.eye(4)):
        self.rotation_matrix = rotation_matrix
        
    def apply_rotation(self, rotation_matrix = None):
        if rotation_matrix is not None:
            self.set_rotation(rotation_matrix)
        if self.rotation_matrix is None:
            raise ValueError("Rotation matrix not set.")
        self.rotated_mesh = self.original_mesh.copy()
        self.rotated_mesh.apply_transform(self.rotation_matrix)

    def get_rotation(self):
        return self.rotation_matrix
    
    def display_rotation(self, scale=0.10):
        '''        
        Displays the rotation axes according to the rotation matrix.
        The rotation matrix is a 4x4 matrix, the last row is ignored.
        The first three columns are the x, y, z axes respectively.
        The last column is the translation vector.
        The rotation matrix is applied to the original mesh and the axes are displayed.
        The axes are scaled by the scale parameter.
        '''
        if self.rotation_matrix is None:
            print("No rotation matrix set.")
            return

        R = self.rotation_matrix[:3, :3]  # Extract 3x3 rotation part
        origin = np.array([0, 0, 0])

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        # Draw original axes
        ax.quiver(*origin, scale, 0, 0, color='r', linestyle='dashed', label='X original')
        ax.quiver(*origin, 0, scale, 0, color='g', linestyle='dashed', label='Y original')
        ax.quiver(*origin, 0, 0, scale, color='b', linestyle='dashed', label='Z original')
        # Draw transformed axes
        ax.quiver(*origin, *R[:, 0] * scale, color='r', label='X transformed')
        ax.quiver(*origin, *R[:, 1] * scale, color='g', label='Y transformed')
        ax.quiver(*origin, *R[:, 2] * scale, color='b', label='Z transformed')

        ax.set_box_aspect(aspect = [1,1,1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"Rotation Axes for {self.name}")
        ax.legend()
        plt.show()

    def set_vertex_function(self, function, inverse_function):
        self.vertex_function = function
        self.vertex_inverse_function = inverse_function
    
    def apply_vertex_function(self,function = None, inverse_function = None):
        if function is not None:
            self.set_vertex_function(function)
        if self.vertex_function is None:
            raise ValueError("Vertex function not set.")
        base_mesh = self.original_mesh.copy()
        if self.rotated_mesh is not None:
            base_mesh = self.rotated_mesh.copy()
        base_mesh.vertices = self.vertex_function(base_mesh.vertices)
        self.transformed_mesh = base_mesh

    
    def slice_mesh(self, step = 0.2, z_start=None, z_end=None):
        mesh = self.get_mesh().copy()
        if z_start is None:
            z_start = mesh.bounds[0][2]
        if z_end is None:
            z_end = mesh.bounds[1][2]
        slices = []
        for z in np.arange(z_start, z_end, step):
            slice_mesh = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if slice_mesh is not None:            
                slices.append({
                    'z': z,
                    'origin': [0, 0, z],
                    'normal': [0, 0, 1],
                    'section': slice_mesh
                })
        self.slices = slices
        return slices


    def display_transformed(self):
        mesh = self.get_mesh().copy()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        mesh_faces = mesh.vertices[mesh.faces]

        ax.add_collection3d(Poly3DCollection(mesh_faces, facecolors='cyan', linewidths=0.01, edgecolors='k', alpha=.05))
        #draw slices
        if self.slices is not None:
            for slice_info in self.slices:
                section = slice_info.get("section")
                if section is not None:
                    # Convert section to a Path3D if needed
                    slice_path = section.to_planar()[0].to_3D()
                    for entity in slice_path.entities:
                        points = slice_path.vertices[entity.points]
                        ax.plot(points[:, 0], points[:, 1], points[:, 2], color='orange', alpha=0.2)

        ax.set_box_aspect(aspect = [1,1,1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f"Transformed Mesh for {self.name}")
        plt.show()
    

    def point_cloud_from_mesh(self, n_points=2000, seed=42):
        mesh = self.get_mesh().copy()
        if seed is not None:
            np.random.seed(seed)
        self.point_cloud = mesh.sample(n_points)
        return self.point_cloud
    def get_point_cloud(self, regen = False):
        if self.point_cloud is None or regen:
            self.point_cloud_from_mesh()
        return self.point_cloud


    def __repr__(self):
        return (
            f"<MeshSample '{self.name}': "
            f"transformed={'yes' if self.transformed_mesh is not None else 'no'}, "
            f"slices={len(self.slices)}, "
            f"evaluated={'yes' if self.evaluation else 'no'}>"
        )
