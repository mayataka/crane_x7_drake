# use as: blender --background --python stl_to_obj.py
import bpy
import glob


for stl_file in glob.glob('./meshes/visual/*.stl', recursive=True):
    obj_file = stl_file.replace('stl', 'obj')
    bpy.ops.import_mesh.stl(filepath=stl_file, axis_forward='-Z', axis_up='Y')
    bpy.ops.export_scene.obj(filepath=obj_file, axis_forward='-Z', axis_up='Y', use_selection=True, use_materials=False)

for stl_file in glob.glob('./meshes/collision/*.stl', recursive=True):
    obj_file = stl_file.replace('stl', 'obj')
    bpy.ops.import_mesh.stl(filepath=stl_file, axis_forward='-Z', axis_up='Y')
    bpy.ops.export_scene.obj(filepath=obj_file, axis_forward='-Z', axis_up='Y', use_selection=True, use_materials=False)