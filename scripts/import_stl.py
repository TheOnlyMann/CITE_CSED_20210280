import numpy as np
import trimesh
import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import prusa_slicer as ps

# Function to import a mesh from a file
def import_stl(file_path, fix_stl = False):
    #check if it does not have .stl extension and if not add it
    if not file_path.endswith(".stl"):
        file_path += ".stl"
    #check if the file exists in the current directory, if not check in the base data/stl directory
    if not os.path.exists(file_path):
        file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data/stl", file_path)
    #with all that done check if the file exists, if not print an error message and exit
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    print(f"✅ STL loaded from: {file_path}")
    if fix_stl:
        #repair the stl file using prusa slicer
        ps.repairSTL(file_path)
        print("✅ STL repaired")
    return trimesh.load(file_path)#load the mesh using trimesh

# Function to plot a mesh along with information, deprecated
def mesh_plot_info(mesh, title=""):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    mesh_faces = mesh.vertices[mesh.faces]
    ax.add_collection3d(Poly3DCollection(mesh_faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))
    ax.set_box_aspect(aspect = [1,1,1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    if title != "":
        ax.set_title(title)
    plt.show()