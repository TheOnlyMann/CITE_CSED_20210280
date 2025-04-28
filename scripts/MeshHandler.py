import trimesh
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
from trimesh.geometry import align_vectors

from STLBase import STLBase

class MeshHandler:
    def __init__(self, name: str = "unnamed"):
        self.name = name
        self.original_mesh = STLBase(name).load()
        self.rotation_matrix = None
        self.rotated_mesh = None
        self.transformed_mesh = None

    
    def set_rotation(self, rotation_matrix = None, angles_rad = None, remove_Z_rotation = True):
        '''
        Set the rotation matrix for the mesh.
        Input can be either a 4x4 rotation matrix or a list of angles in radians.
        The angles are assumed to be in the order of [X, Y, Z] rotation.
        If remove_Z_rotation is True, the Z rotation will be set to 0.
        The rotation matrix is applied to the original mesh and the rotated mesh is stored.
        '''

        if angles_rad is not None:
            angles_rad = list(angles_rad)# make it local
            if len(angles_rad) < 2:
                raise ValueError("At least two angles are required for rotation.")
            elif len(angles_rad) > 3:
                raise ValueError("Only maximum of three angles are required for rotation.")
            elif len(angles_rad) == 2:
                angles_rad.append(0.0)
            if remove_Z_rotation:
                angles_rad[2] = 0.0
            angle_x, angle_y, angle_z = angles_rad
            rotation_matrix = trimesh.transformations.euler_matrix(angle_x, angle_y, angle_z, axes='sxyz')

        elif rotation_matrix is not None:
            # Check if the rotation matrix is valid
            if not isinstance(rotation_matrix, np.ndarray) or rotation_matrix.shape != (4, 4):
                raise ValueError("Rotation matrix must be a 4x4 numpy array.")
            #check if the rotation matrix is orthogonal and determinant is 1
            R = rotation_matrix[:3, :3]
            if not np.allclose(np.dot(R, R.T), np.eye(3), atol=1e-6):
                raise ValueError("Rotation matrix must be orthogonal.")
            if not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
                raise ValueError("Rotation matrix must have determinant 1.")
            
            if remove_Z_rotation:
                angle_x, angle_y, _ = trimesh.transformations.euler_from_matrix(rotation_matrix, axes='sxyz')
                rotation_matrix = trimesh.transformations.euler_matrix(angle_x, angle_y, 0.0, axes='sxyz')

        else:
            rotation_matrix = np.eye(4)#default to identity matrix
        self.rotation_matrix = rotation_matrix
        
    def apply_rotation(self):
        '''
        apply the rotation matrix to the original mesh and store the rotated mesh.
        If the rotation matrix is not set, it will raise a ValueError.
        while it is recommended to set the rotation matrix on set_rotation method, you can also set it here.
        If the rotation matrix is not set, it will raise a ValueError.
        '''
        if self.original_mesh is None:
            raise ValueError("Original mesh not set.")
        if self.rotation_matrix is None:
            raise ValueError("Rotation matrix not set.")
        self.rotated_mesh = self.original_mesh.copy()
        self.rotated_mesh.apply_transform(self.rotation_matrix)

    def set_rotation_by_base(self, base, remove_Z_rotation = True):
        '''
        Set the rotation matrix based on the selected face within the stl.
        The face selected as the base will be rotated to the XY plane, making the Z axis normal to the face.
        The rotation matrix is applied to the original mesh and the rotated mesh is stored.
        '''
        if not isinstance(base, int) or base >= len(self.original_mesh.faces):
            raise ValueError("Base must be a valid face index.")

        # Get normal of the selected face
        face_normal = self.original_mesh.face_normals[base]

        # Desired normal (Z-axis)
        target_normal = [0, 0, 1]

        # Compute rotation matrix to align face_normal with Z-axis
        rotation_matrix = align_vectors(face_normal, target_normal)

        # Make it 4x4 homogeneous
        transform = trimesh.transformations.identity_matrix()
        transform[:3, :3] = rotation_matrix

        # Save
        self.rotation_matrix = self.set_rotation(rotation_matrix=transform, remove_Z_rotation=remove_Z_rotation)
        

    def get_rotation(self):
        '''
        Returns the rotation matrix.
        If the rotation matrix is not set, it will return None.
        '''
        if self.rotation_matrix is None:
            return None
        return self.rotation_matrix
    
    def get_rotation_angles(self):
        '''
        Returns the rotation angles in radians.
        The angles are in the order of [X, Y, Z] rotation.
        If the rotation matrix is not set, it will return None.
        '''
        if self.rotation_matrix is None:
            return None
        angles = trimesh.transformations.euler_from_matrix(self.rotation_matrix, axes='sxyz')
        return angles
    
    def display_rotation(self, scale=0.10):
        '''        
        Displays the rotation axes according to the rotation matrix.
        The rotation matrix is a 4x4 matrix, the last row is ignored.
        The first three columns are the x, y, z axes respectively.
        The last column is the translation vector.
        The rotation matrix is applied to the original mesh and the axes are displayed.
        The axes are scaled by the scale parameter.
        also display their X, Y, Z rotation angles in degrees.
        '''
        if self.rotation_matrix is None:
            print("No rotation matrix set.")
            return

        # Extract Euler angles
        angles_rad = trimesh.transformations.euler_from_matrix(self.rotation_matrix, axes='sxyz')
        angles_deg = np.degrees(angles_rad)

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')

        origin = np.array([0, 0, 0])

        # Original axes (dashed)
        ax.quiver(*origin, scale, 0, 0, color='gray', linestyle='dashed', alpha=0.5)
        ax.quiver(*origin, 0, scale, 0, color='gray', linestyle='dashed', alpha=0.5)
        ax.quiver(*origin, 0, 0, scale, color='gray', linestyle='dashed', alpha=0.5)

        # Transformed axes
        R = self.rotation_matrix[:3, :3]
        ax.quiver(*origin, *(R[:, 0] * scale), color='r', label='X rotated')
        ax.quiver(*origin, *(R[:, 1] * scale), color='g', label='Y rotated')
        ax.quiver(*origin, *(R[:, 2] * scale), color='b', label='Z rotated')

        # Set plot annotations (angles)
        angle_text = (f"Rotation angles:\n"
                    f"X: {angles_rad[0]:.2f} rad ({angles_deg[0]:.1f}°)\n"
                    f"Y: {angles_rad[1]:.2f} rad ({angles_deg[1]:.1f}°)\n"
                    f"Z: {angles_rad[2]:.2f} rad ({angles_deg[2]:.1f}°)")
        
        # Place text box in plot
        ax.text2D(0.05, 0.95, angle_text, transform=ax.transAxes,
                fontsize=12, bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7))

        # Axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_title(f"Rotation Visualization: {self.name}")
        ax.set_box_aspect([1, 1, 1])
        ax.legend()
        plt.tight_layout()
        plt.show()


    def set_vertex_function(self, function, inverse_function):
        '''set the vertex function and inverse function for the mesh.
        The function is just a linear z transformation, but it can be any function that takes a 3D point and returns a 3D point.
        The inverse function is used to get back to the original mesh.
        The function and inverse function should be of the form f(x,y,z) = (x',y',z') where (x,y,z) is the original point and (x',y',z') is the transformed point.
        '''
        self.vertex_function = function
        self.vertex_inverse_function = inverse_function
    
    def apply_vertex_function(self,function = None, inverse_function = None):
        '''
        apply the vertex function to the mesh.
        The function should be of the form f(x,y,z) = (x',y',z') where (x,y,z) is the original point and (x',y',z') is the transformed point.
        The inverse function is used to get back to the original mesh.
        '''
        if function is not None:
            self.set_vertex_function(function)
        if self.vertex_function is None:
            raise ValueError("Vertex function not set.")
        if self.rotated_mesh is None:
            raise ValueError("Rotated mesh not set.")
        base_mesh = self.rotated_mesh.copy()
        base_mesh.vertices = self.vertex_function(base_mesh.vertices)
        self.transformed_mesh = base_mesh

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

    def save_stl(self, filename):
        '''
        Save the mesh to a file.
        If the mesh is not set, it will raise a ValueError.
        '''
        mesh = self.get_mesh()
        if mesh is None:
            raise ValueError("No mesh set.")
        # Save the mesh to a file
        mesh.export(filename)
        print(f"Mesh saved to {filename}")

    def evaluate(self, evaluation_function):
        '''
        Evaluate the mesh using the given evaluation function.
        The evaluation function should take a mesh and return a score.
        The score is stored in the evaluation attribute of the object.
        '''
        if self.transformed_mesh is None:
            raise ValueError("Transformed mesh not set.")
        if evaluation_function is None:
            raise ValueError("Evaluation function not set.")
        self.evaluation = evaluation_function(self.transformed_mesh)
        return self.evaluation
    
    def get_evaluation(self):
        '''
        Returns the evaluation score.
        If the evaluation score is not set, it will return None.
        '''
        if self.evaluation is None:
            return None
        return self.evaluation
    

    def __repr__(self):
        return (
            f"<MeshSample '{self.name}': "
            f"transformed={'yes' if self.transformed_mesh is not None else 'no'}, "
            f"slices={len(self.slices)}, "
            f"evaluated={'yes' if self.evaluation else 'no'}>"
        )
