import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from import_stl import import_stl

class STLBase:
    def __init__(self, filename: str, mesh: trimesh.Trimesh = None, **kwargs):
        self.filename = filename
        self.mesh = mesh
        self.face_tag = None
    
    def _check(self):
        if self.mesh is None:
            raise ValueError("No mesh loaded.")

    def load(self, filename:str  = None):
        if filename is None:
            filename = self.filename
        self.mesh = import_stl(filename)
        return self.mesh

    def set(self, mesh: trimesh.Trimesh):
        self.mesh = mesh
    def get(self):
        self._check()
        return self.mesh
    def get_copy(self):
        self._check()
        return self.mesh.copy()
    
    def save(self, filename: str = None):
        self._check()
        if filename is None:
            filename = self.filename + "_processed"
        #check if it does not have .stl extension and if not add it
        if not filename.endswith(".stl"):
            filename += ".stl"
        #check if the file exists in the current directory, if not check in the base data/stl directory
        if not os.path.exists(file_path):
            file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data/stl", filename)

        self.mesh.export(filename)
        print(f"✅ STL saved to: {filename}")
    
    def display(self, title: str = ""):
        '''
        Displays the original mesh in 3D.
        The mesh is displayed with a cyan color and a black edge.
        If the mesh is not set, it will print it out and return
        '''
        mesh = self.get_copy()
        if title == "":
            title = self.filename
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        mesh_faces = mesh.vertices[mesh.faces]
        ax.add_collection3d(Poly3DCollection(mesh_faces, facecolors='cyan', linewidths=0.5, edgecolors='k', alpha=.25))
        ax.set_box_aspect(aspect = [1,1,1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title)
        plt.show()
        


        