import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
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
        self.rotation_matrix = np.eye(4)
        self.rotation_angles = np.zeros(3)
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
        rotation_matrix: np.ndarray = None,
        bottom_selection: int = None,
        limit_z_rotation: bool = True
    ):
        '''
        Sets the rotation for the mesh in one of three ways:
        '''
        inputs = [rotation_angles, rotation_matrix, bottom_selection]
        if sum(x is not None for x in inputs) != 1:
            raise ValueError("Only one of rotation_angles, rotation_matrix, or bottom_selection must be set.")

        mesh = self.inputfile.get()
        
        if rotation_angles is not None:
            angles = np.array(rotation_angles)
            if limit_z_rotation:
                angles[2] = 0.0
            R = trimesh.transformations.euler_matrix(*angles, axes='sxyz')
            self.rotation_matrix = R
            self.rotation_angles = angles

        elif rotation_matrix is not None:
            if not isinstance(rotation_matrix, np.ndarray) or rotation_matrix.shape != (4, 4):
                raise ValueError("rotation_matrix must be a 4x4 numpy array.")
            if limit_z_rotation:
                angles = trimesh.transformations.euler_from_matrix(rotation_matrix, axes='sxyz')
                angles = [angles[0], angles[1], 0.0]
                R = trimesh.transformations.euler_matrix(*angles, axes='sxyz')
            else:
                R = rotation_matrix
            self.rotation_matrix = R
            self.rotation_angles = trimesh.transformations.euler_from_matrix(R, axes='sxyz')

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
                R = np.eye(4)
            else:
                axis = axis / np.linalg.norm(axis)
                R = trimesh.transformations.rotation_matrix(angle, axis)

            if limit_z_rotation:
                angles = trimesh.transformations.euler_from_matrix(R, axes='sxyz')
                angles = [angles[0], angles[1], 0.0]
                R = trimesh.transformations.euler_matrix(*angles, axes='sxyz')
            self.rotation_matrix = R
            self.rotation_angles = trimesh.transformations.euler_from_matrix(R, axes='sxyz')
            

    def apply_rotation(self, rotation_matrix: np.ndarray = None):
        '''
        Syncs the rotation matrix with the STL file.
        '''
        if rotation_matrix is not None:
            self.rotation_matrix = rotation_matrix
        self.outputfile.set(self.inputfile.get().copy())
        self.outputfile.get().apply_transform(self.rotation_matrix)

    def display(self, title: str = ""):
        title + f"({self.rotation_angles[0]:.2f}, {self.rotation_angles[1]:.2f}, {self.rotation_angles[2]:.2f})"
        self.outputfile.display(title)
    
        


        