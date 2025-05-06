import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.transformations import quaternion_matrix, quaternion_from_euler, quaternion_about_axis, euler_from_matrix

from scripts.import_stl import import_stl
from scripts.STLBase import STLBase

class STLRotater:
    '''
    Class for rotating STL files.
    It uses the trimesh library to load and manipulate the STL files.
    It can rotate the mesh around the X, Y and Z axes.
    It can also save the mesh to a new STL file.
    It can also display the mesh in 3D.
    ''' 
    def __init__(self, filename: str = "placeholder", mesh: trimesh.Trimesh = None, **kwargs):
        self.inputfile = STLBase(filename, mesh, **kwargs)
        self.outputfile = STLBase(filename, mesh, **kwargs)
        self.rotation_angles: np.ndarray = None
        self.quaternion: np.ndarray = None
        self.bottom_selection = None#face number of the bottom face of the mesh
    
    def _check(self):
        self.inputfile._check()

    def load(self, filename:str  = None):
        return self.inputfile.load(filename)

    def set(self, mesh: trimesh.Trimesh):
        self.inputfile.set(mesh)
    def get(self):
        return self.outputfile.get()
    def get_copy(self):
        return self.outputfile.get_copy()
    def transfer(self):
        return self.outputfile.transfer()
    
    def save(self, filename: str = None):
        self.outputfile.save(filename)
        
    def set_rotation(
        self,
        rotation_angles: np.ndarray = None,
        quaternion: np.ndarray = None,
        bottom_selection: int = None,
        limit_z_rotation: bool = True
    ):
        """
        Set rotation using one of the three options:
        - Euler angles (in radians)
        - Quaternion [x, y, z, w]
        - Align face normal to Z+
        """
        mesh = self.inputfile.get()
        inputs = [rotation_angles, quaternion, bottom_selection]
        if sum(x is not None for x in inputs) != 1:
            raise ValueError("Only one of rotation_angles, quaternion, or bottom_selection must be set.")

        if rotation_angles is not None:
            angles = np.array(rotation_angles)
            if limit_z_rotation:
                angles[2] = 0.0
            quat = quaternion_from_euler(*angles, axes='sxyz')
            self.quaternion_matrix = quaternion_matrix(quat)
            self.rotation_angles = angles

        elif quaternion is not None:
            quat = np.array(quaternion)
            self.quaternion_matrix = quaternion_matrix(quat)
            self.rotation_angles = euler_from_matrix(self.quaternion_matrix)

        elif bottom_selection is not None:
            if bottom_selection >= len(mesh.faces):
                raise IndexError("bottom_selection index out of range.")
            face = mesh.faces[bottom_selection]
            verts = mesh.vertices[face]
            normal = trimesh.geometry.normal_triangle(*verts)
            z_axis = np.array([0, 0, 1])
            axis = np.cross(normal, z_axis)
            angle = np.arccos(np.clip(np.dot(normal, z_axis), -1.0, 1.0))
            if np.linalg.norm(axis) < 1e-6:
                quat = [0, 0, 0, 1]
            else:
                axis = axis / np.linalg.norm(axis)
                quat = quaternion_about_axis(angle, axis)

            if limit_z_rotation:
                mat = quaternion_matrix(quat)
                angles = euler_from_matrix(mat)
                angles = [angles[0], angles[1], 0.0]
                quat = quaternion_from_euler(*angles)
            self.quaternion_matrix = quaternion_matrix(quat)
            self.rotation_angles = euler_from_matrix(self.quaternion_matrix)
            

    def apply_rotation(self):
        '''
        Syncs the rotation matrix with the STL file.
        '''
        self.outputfile.set(self.inputfile.get().copy())
        self.outputfile.get().apply_transform(self.quaternion_matrix)

    def display(self, title: str = ""):
        title_extra =  f"({self.rotation_angles[0]:.2f}, {self.rotation_angles[1]:.2f}, {self.rotation_angles[2]:.2f})"
        self.outputfile.display(title + title_extra)
    
        


        