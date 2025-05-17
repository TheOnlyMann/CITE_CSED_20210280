import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.transformations import quaternion_matrix, quaternion_from_euler, quaternion_about_axis, euler_from_matrix
from shapely.geometry import Polygon

from scripts.import_stl import import_stl
from scripts.STLBase import STLBase
from scripts.prusa_slicer import sliceSTL

def warpmethod_1(x, y):
    # Example warp method: a simple sine wave
    return 0.5 * np.cos(0.2 * x) + 0.5 * np.cos(0.2 * y)
class STLWarper:
    '''
    Class for making STL file warp to a certain shape, and back to the original shape when its converted into GCODE.
    It uses the trimesh library to load and manipulate the STL files.
    ''' 
    def __init__(self, filename: str = "placeholder", mesh: trimesh.Trimesh = None, **kwargs):
        self.inputfile = STLBase(filename, mesh, **kwargs)
        self.preprocess_mesh = None
        self.outputfile = STLBase(filename, mesh, **kwargs)
        self.center_offset = None
        self.warp_method = None
        self.outputgcode = None

    
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
    def preprocess(self):
        mesh = self.inputfile.get_copy()
        mesh = mesh.subdivide_to_size(max_edge=1.0)
        mesh.remove_degenerate_faces()
        mesh.remove_duplicate_faces()
        mesh.remove_infinite_values()
        mesh.remove_unreferenced_vertices()
        mesh.rezero()          # Z=0 중심 재정렬
        mesh.fix_normals() 
        self.preprocess_mesh = mesh
        self.center_offset = mesh.bounds[0] + (mesh.bounds[1] - mesh.bounds[0]) / 2
        print(f"Mesh preprocessed. Center offset: {self.center_offset}")
        print(f"Mesh bounds: {self.preprocess_mesh.bounds}")

    def set_warp_method(self, method):
        if method is None:
            raise ValueError("Warp method cannot be None.")
        if not callable(method):
            raise TypeError("Warp method must be a callable function.")
        self.warp_method = method

    def warp(self):
        if self.preprocess_mesh is None:
            raise RuntimeError("Call preprocess() before warp().")
        if self.warp_method is None:
            raise RuntimeError("Set warp_method before calling warp().")

        mesh = self.preprocess_mesh.copy()
        verts = mesh.vertices.copy()

        for i in range(len(verts)):
            x, y, z = verts[i]
            x -= self.center_offset[0]
            y -= self.center_offset[1]
            delta_z = self.warp_method(x, y)
            verts[i][2] = z + delta_z

        mesh.vertices = verts
        mesh.remove_degenerate_faces()
        mesh.remove_duplicate_faces()
        mesh.remove_infinite_values()
        mesh.remove_unreferenced_vertices()
        mesh.rezero()          # Z=0 중심 재정렬
        mesh.fix_normals() 
        self.outputfile.set(mesh)

    def slice_and_reverse(self, **kwargs):
        if self.outputfile is None:
            raise RuntimeError("Call warp() before slice_and_return().")
        if self.outputfile.get() is None:
            raise RuntimeError("Output file is empty.")
        self.outputfile.save("temp/warped")
        sliceSTL("data/stl/temp/warped.stl", outputPath="data/gcode/temp")
        self.outputgcode = read_and_warp_gcode("data/gcode/temp/warped.gcode", self.warp_method)
        


    def display(self, title: str = ""):
        title_extra =  f""
        #use the trimesh library to display the mesh instead since it is more efficient
        if self.outputfile.get() is not None:
            mesh = self.outputfile.get()
            if title != "":
                title_extra = f" - {title}"
            mesh.show(title=title + title_extra)
    
        


def read_and_warp_gcode(filename: str, warp_method: callable, interpolate_size = 0.5):
    '''
    Reads a GCode file, warps the STL file using the specified warp method, and saves the warped STL file.
    :param filename: Path to the GCode file that we will save. original gcode will always be in data/gcode/temp/warped.gcode
    :param warp_method: Function to warp the STL file. It should take x and y coordinates as input and return the z displacement.
    :return: Path to the warped STL file, saved in proper directory.
    '''
    filepath = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data/gcode/temp/warped.gcode")
    with open(filepath, 'r') as f_gcode:
        lines = f_gcode.readlines()
    
    x = y = z = e = f = 0
    prev_x = prev_y = prev_z = 0
    ignore = False
    z_min = 0.13
    #save the modified lines to a new file
    warped_filename = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data/gcode/" + os.path.basename(filename))
    with open(warped_filename, 'w') as f_warped:
        for line in lines:
            if line.startswith(";WIPE_START"):
                ignore = True
            elif line.startswith(";WIPE_END"):
                ignore = False
            if ignore:
                continue
            if line.startswith("G1"):
                parts = line.strip().split()
                e = 0
                f = 0
                for part in parts:
                    if part.startswith("X"):
                        prev_x = x
                        x = float(part[1:])
                    elif part.startswith("Y"):
                        prev_y = y
                        y = float(part[1:])
                    elif part.startswith("Z"):
                        prev_z = z
                        z = float(part[1:])
                    elif part.startswith("E"):
                        e = float(part[1:])
                    elif part.startswith("F"):
                        f = float(part[1:])
                new_z = z -1 * warp_method(x-110, y-110)
                if new_z < z_min:
                    new_z = z_min
                #calculate the distance between the current and previous point, and if it is greater than the interpolate size, interpolate between the two points
                distance = np.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2 + (new_z - prev_z) ** 2)
                if distance > interpolate_size:
                    num_interpolations = int(distance / interpolate_size)
                    for i in range(num_interpolations):
                        t = (i + 1) / (num_interpolations + 1)
                        new_x = prev_x + t * (x - prev_x)
                        new_y = prev_y + t * (y - prev_y)
                        new_new_z = prev_z + t * (new_z - prev_z) - 1 * warp_method(new_x-110, new_y-110)
                        new_e = e / (num_interpolations + 1)
                        new_f = f
                        new_line = f"G1 X{new_x:.3f} Y{new_y:.3f} Z{new_new_z:.3f} "
                        if new_e != 0:
                            new_line += f"E{new_e:.3f} "
                        if new_f != 0:
                            new_line += f"F{new_f:.3f} "
                        new_line += "\n"
                        f_warped.write(new_line)
                else:
                    new_line = f"G1 X{x:.3f} Y{y:.3f} Z{new_z:.3f} "
                    if e != 0:
                        new_line += f"E{e:.3f} "
                    if f != 0:
                        new_line += f"F{f:.3f} "
                    new_line += "\n"
                    f_warped.write(new_line)
            else:
                f_warped.write(line)

    return warped_filename