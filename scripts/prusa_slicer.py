import subprocess
import os

SLICER_PATH = "C:\Program Files\Prusa3D\PrusaSlicer"
project_dir = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PROFILE = os.path.join(project_dir,"..\data\slicer\profiles\ender3_se.ini")
DEFAULT_PARAMS = ""

# Slices a given STL-File planar in prusa slicer
# ----------------------------------------
# Input: target: STL-File path (string), profile: INI-File path (string), userParameters: additional paramters (string)
# Output: None
def sliceSTL(target, profile=DEFAULT_PROFILE, userParameters = DEFAULT_PARAMS, userPath=SLICER_PATH, outputPath=None):
    """
    Slices a given STL-File planar in prusa slicer
    :param target: STL-File path (string), if not specified of actual designation it will search data/stl
    :param profile: INI-File path (string)
    :param userParameters: additional paramters (string)
    :param userPath: path to the prusa slicer installation (string)
    :param outputPath: path to the output directory (string), if not specified it will save in data/gcode
    :return: None
    """
    if not os.path.exists(target):
        target = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data/stl", target)
        filename = os.path.join(os.path.dirname(target).replace("\stl","\gcode"), os.path.basename(target).replace(".stl", ".gcode"))
    else:
        filename = os.path.join(os.path.dirname(target), os.path.basename(target).replace(".stl", ".gcode"))
    if outputPath is not None:
        filename = os.path.join(outputPath, os.path.basename(target).replace(".stl", ".gcode"))
    prusa_slicer_path = fr'"{userPath}\prusa-slicer-console.exe"'
    command = f'{prusa_slicer_path} --export-gcode {target} --output {filename} --load {profile} {userParameters}'
    print(command)
    subprocess.run(command, shell=True)

# Repairs any faults in the STL-Facet normals
# ----------------------------------------
# Input: target: STL-File path (string)
# Output: None
def repairSTL(target):
    command = f'prusa-slicer-console.exe --export-stl {target} --repair --loglevel 0 --output {target}'
    print(command)
    subprocess.run(command, shell=True,stdout=subprocess.DEVNULL,
    stderr=subprocess.STDOUT)

# Opens the given STL-File in Prusa GCode-Viewer
# ----------------------------------------
# Input: target: GCode-File path (string)
# Output: None
def viewGCODE(target, userPath=SLICER_PATH):
    prusa_path = fr'"{userPath}\prusa-gcodeviewer.exe"'
    command = f"{prusa_path} {target}"
    print(command)
    subprocess.run(command, shell=True)
