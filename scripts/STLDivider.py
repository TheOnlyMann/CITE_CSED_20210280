import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.transformations import quaternion_matrix, quaternion_from_euler, quaternion_about_axis, euler_from_matrix
from trimesh.intersections import slice_mesh_plane

from scripts.import_stl import import_stl
from scripts.STLBase import STLBase

class STLDivider:
    '''
    Class for dividing STL files.
    It uses the trimesh library to load and manipulate the STL files.
    Given a z_height, it divides the mesh into two parts:
    - The part above the z_height
    - The part below the z_height
    It can also save the mesh to a new STL file, respectively.
    It can also display the mesh in 3D.
    ''' 
    def __init__(self, filename: str = "placeholder", mesh: trimesh.Trimesh = None, **kwargs):
        self.inputfile = STLBase(filename, mesh, **kwargs)
        self.outputfile_upper = STLBase(filename, mesh, **kwargs)
        self.outputfile_lower = STLBase(filename, mesh, **kwargs)
        self.z_height = None
        self.z_limit = (self.inputfile.get().bounds[0][2],self.inputfile.get().bounds[1][2]) #z limit of the mesh, used for checking if the mesh is above or below the z_height

    
    def _check(self):
        self.inputfile._check()

    def load(self, filename:str  = None):
        return self.inputfile.load(filename)

    def set(self, mesh: trimesh.Trimesh):
        self.inputfile.set(mesh)
    def get(self, mesh_type="upper"):
        if mesh_type == "upper":
            return self.outputfile_upper.get()
        elif mesh_type == "lower":
            return self.outputfile_lower.get()
        else:
            raise ValueError("mesh_type must be 'upper' or 'lower'")
    def get_copy(self, mesh_type="upper"):
        if mesh_type == "upper":
            return self.outputfile_upper.get_copy()
        elif mesh_type == "lower":
            return self.outputfile_lower.get_copy()
        else:
            raise ValueError("mesh_type must be 'upper' or 'lower'")
    def transfer(self, mesh_type="upper"):
        if mesh_type == "upper":
            return self.outputfile_upper.transfer()
        elif mesh_type == "lower":
            return self.outputfile_lower.transfer()
        else:
            raise ValueError("mesh_type must be 'upper' or 'lower'")
    
    def save(self, filename: str = None, mesh_type="upper"):
        if mesh_type == "upper":
            self.outputfile_upper.save(filename)
        elif mesh_type == "lower":
            self.outputfile_lower.save(filename)
        else:
            raise ValueError("mesh_type must be 'upper' or 'lower'")
        
    def get_z_limit(self):
        """
        Get the z limit of the mesh.
        """
        return self.z_limit
        
    def set_z_height(self, z_height: float):
        """
        Set the z_height for dividing the mesh.
        """
        self.z_height = z_height
        if self.z_height is None:
            raise ValueError("z_height must be set before dividing the mesh.")
        elif self.z_height < self.z_limit[0] or self.z_height > self.z_limit[1]:
            print(f"Warning: z_height {self.z_height} is outside the bounds of the mesh {self.z_limit}.")
        else:
            print(f"z_height set to {self.z_height}")
    
    def divide(self):
        """
        Divide the mesh into two parts: upper and lower at self.z_height.
        The lower mesh is flipped upside down.
        """
        if self.z_height is None:
            raise ValueError("z_height must be set before dividing the mesh.")
        elif not (self.z_limit[0] <= self.z_height <= self.z_limit[1]):
            print(f"⚠️ z_height {self.z_height} out of bounds {self.z_limit}")

        mesh = self.inputfile.get()

        plane_origin = np.array([0, 0, self.z_height])
        plane_normal = np.array([0, 0, 1])

        try:
            upper = slice_mesh_plane(
                mesh,
                plane_origin=plane_origin,
                plane_normal=-plane_normal,
                cap=True
            )
            lower = slice_mesh_plane(
                mesh,
                plane_origin=plane_origin,
                plane_normal=plane_normal,
                cap=True
            )
        except Exception as e:
            print(f"❌ Mesh splitting failed: {e}")
            return
        
        if upper is None or lower is None:
            print("❌ Slicing returned empty mesh.")
            return

        # Flip the upper mesh 180° around X-axis
        R = trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
        upper.apply_transform(R)

        self.outputfile_upper.set(upper)
        self.outputfile_lower.set(lower)
        print(f"✅ Mesh divided at z={self.z_height:.2f}.")
        
    def display(self, mesh_type=None, window_name="STL Divider"):
        """
        Display the mesh using matplotlib.
        """
        if mesh_type == "upper":
            self.outputfile_upper.display(title=window_name)
            return
        elif mesh_type == "lower":
            self.outputfile_lower.display(title=window_name)
            return
        elif mesh_type != None:
            raise ValueError("mesh_type must be 'upper' or 'lower'")
        mesh = self.inputfile.get_copy()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        mesh_faces = mesh.vertices[mesh.faces]
        ax.add_collection3d(Poly3DCollection(mesh_faces, facecolors='cyan', linewidths=0.5, edgecolors='k', alpha=.25))
        ax.set_box_aspect(aspect = [1,1,1])

        #get the bounds of the mesh
        bounds = mesh.bounds
        x_min, x_max = bounds[0][0], bounds[1][0]
        y_min, y_max = bounds[0][1], bounds[1][1]
        # Draw z_height 
        if self.z_height is not None:
            z = self.z_height

            # Draw a horizontal plane at z_height
            plane_verts = [
                [x_min, y_min, z],
                [x_max, y_min, z],
                [x_max, y_max, z],
                [x_min, y_max, z]
            ]

            # Add the plane to the plot
            ax.add_collection3d(Poly3DCollection([plane_verts], color='red', alpha=0.2))

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.axis('equal')
        ax.set_title(f"{window_name} - z_height: {self.z_height:.2f}")
        plt.show()
    
        


        