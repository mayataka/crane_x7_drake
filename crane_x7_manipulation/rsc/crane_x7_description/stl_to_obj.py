import glob
import pymeshlab
from os.path import abspath


for stl_file in glob.glob('./meshes/visual/*.stl', recursive=False):
    obj_file = stl_file.replace('stl', 'obj')
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(abspath(stl_file))
    # ms.convex_hull()
    ms.save_current_mesh(abspath(obj_file))

for stl_file in glob.glob('./meshes/collision/*.stl', recursive=False):
    obj_file = stl_file.replace('stl', 'obj')
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(abspath(stl_file))
    # ms.convex_hull()
    ms.save_current_mesh(abspath(obj_file))