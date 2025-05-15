import trimesh
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trimesh.transformations import quaternion_matrix, quaternion_from_euler, quaternion_about_axis, euler_from_matrix
from shapely.geometry import Polygon

from scripts.import_stl import import_stl
from scripts.STLBase import STLBase

class STLExtender:
    '''
    Class for making STL file's bottom face more prone to sticking to the build plate.
    It does this by adding a small circle to the corner of the bottom face where it is prone to lifting.
    It uses the trimesh library to load and manipulate the STL files.
    ''' 
    def __init__(self, filename: str = "placeholder", mesh: trimesh.Trimesh = None, **kwargs):
        self.inputfile = STLBase(filename, mesh, **kwargs)
        self.outputfile = STLBase(filename, mesh, **kwargs)
        self.bottom_faces = None#face number of the bottom face of the mesh
        self.disc_radius = 5 # radius of the disc, only placed on bottom faces so uses 0.5mm radius
        self.disc_height = 0.2 # height of the disc, only placed on bottom faces so uses 0.2mm layer height
        self.disc_threshold = 90  # only placed on bottom faces if meets a certain angle threshold, uses degrees
        self.disc_area_threshold = 1e-4 # area threshold for the disc, only placed on bottom faces if meets a certain area threshold
        self.disc_center = None # center of the disc, will be a array 

    
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

    def set_disc_state(
        self,
        disc_radius: float = 5,
        disc_height: float = 0.2,
        disc_threshold: float = 90.0,
        disc_area_threshold: float = 1e-4,
    ):
        """
        Set the state of the disc.
        """
        self.disc_radius = disc_radius
        self.disc_height = disc_height
        self.disc_threshold = disc_threshold
        self.disc_area_threshold = disc_area_threshold

    def find_bottom_selection(self, tolerance=1e-5, angle_threshold=None, area_threshold=None, verbose=True):
        """
        Find candidate points for reinforcement disc placement on the outer edge of the bottom face.
        Discs are placed slightly outward from corners on the outermost contour to act like a brim.

        Parameters:
            tolerance (float): Z slicing tolerance.
            angle_threshold (float): Interior angle threshold in degrees to consider a corner "sharp".
            area_threshold (float): Minimum polygon area to skip center-only disc placement.
            verbose (bool): Print debug info.
        """
        if angle_threshold is not None:
            self.disc_threshold = angle_threshold
        if area_threshold is not None:
            self.disc_area_threshold = area_threshold
        area_threshold = self.disc_area_threshold
        angle_threshold = self.disc_threshold
        self.inputfile._check()
        mesh = self.inputfile.get()
        z_min = mesh.vertices[:, 2].min()
        z_slice = z_min + tolerance

        section = mesh.section(plane_origin=[0, 0, z_slice], plane_normal=[0, 0, 1])
        if section is None:
            raise ValueError("No section found at the given Z plane.")

        paths, to_3D = section.to_2D()
        reinforce_points_set = set()

        if verbose:
            plt.figure(figsize=(6, 6))
            plt.title("Bottom Face Contours")
            ax = plt.gca()

        for entity in paths.entities:
            if entity.metadata.get('is_hole', False):
                continue

            pts = paths.vertices[entity.points]
            if len(pts) < 3:
                continue

            poly = Polygon(pts)
            if not poly.is_valid:
                continue

            ring = poly.exterior
            if not ring.is_ccw:
                # Skip inner hole rings (clockwise)
                continue

            if verbose:
                coords = np.array(ring.coords)
                plt.plot(coords[:, 0], coords[:, 1], 'b-')

            if poly.area < area_threshold:
                center = np.array(poly.centroid.coords[0])
                center_3d = np.array([center[0], center[1], 0])
                cent_3d = trimesh.transform_points([center_3d], to_3D)[0]
                reinforce_points_set.add(tuple(cent_3d))
                if verbose:
                    plt.plot(center[0], center[1], 'ro')
                continue

            pts = np.array(ring.coords[:-1])  # exclude closing point
            n = len(pts)
            offset_distance = self.disc_radius * 0.75  # outward offset

            for i in range(n):
                p_prev = pts[i - 1]
                p_curr = pts[i]
                p_next = pts[(i + 1) % n]

                v1 = p_prev - p_curr
                v2 = p_next - p_curr

                v1 /= np.linalg.norm(v1)
                v2 /= np.linalg.norm(v2)

                dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
                angle = np.degrees(np.arccos(dot))

                if angle < angle_threshold:
                    # Compute outward direction (perpendicular to bisector)
                    bisector = v1 + v2
                    if np.linalg.norm(bisector) < 1e-8:
                        continue
                    bisector /= np.linalg.norm(bisector)
                    outward = np.array([-bisector[1], bisector[0]]) # 90 degrees rotation

                    p_offset = p_curr + outward * offset_distance
                    print(f"Offset point: {p_offset}")
                    p_offset_3d = np.array([p_offset[0], p_offset[1], 0.0])
                    point_3d = trimesh.transform_points([p_offset_3d], to_3D)[0]
                    reinforce_points_set.add(tuple(point_3d))

                    if verbose:
                        plt.plot(p_curr[0], p_curr[1], 'go')         # original corner
                        plt.plot(p_offset[0], p_offset[1], 'ro')     # offset location

        self.reinforce_points = [np.array(p) for p in reinforce_points_set]

        if verbose:
            print(f"✅ Reinforcement points (outer brim corners): {len(self.reinforce_points)}")
            plt.axis('equal')
            plt.show()

    def add_disc(self, verbose=True):
        """
        Add discs to the selected points on the bottom face.
        Discs are merged with the copied input mesh and stored in outputfile.
        """
        if not hasattr(self, 'reinforce_points') or not self.reinforce_points:
            self.outputfile.set(self.inputfile.get())
            if verbose:
                print("❌ No reinforcement points found. No discs added.")
            return

        # Copy of original mesh
        base_mesh = self.inputfile.get_copy()

        disc_meshes = []

        for i, center in enumerate(self.reinforce_points):
            # Create a vertical cylinder centered at origin (bottom at z=0)
            disc = trimesh.creation.cylinder(
                radius=self.disc_radius,
                height=self.disc_height,
                sections=32
            )

            # Move disc base to Z=0 (instead of centered at origin)
            disc.apply_translation([0, 0, self.disc_height / 2.0])

            # Translate disc to the reinforcement point
            disc.apply_translation(center)

            disc_meshes.append(disc)

            if verbose:
                print(f"✅ Disc {i+1} placed at {np.round(center, 3)}")

        # Merge all discs with the original copy mesh
        combined = trimesh.util.concatenate([base_mesh] + disc_meshes)

        # Store the result in outputfile
        self.outputfile.set(combined)
        

    def display(self, title: str = ""):
        title_extra =  f""
        self.outputfile.display(title + title_extra)
    
        


        