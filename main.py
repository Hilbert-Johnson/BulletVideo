import os
from c_urdf import convert2urdf
from render import render
from time import sleep

for i in range(50):
    input_obj_name = os.path.join("./tmp", "cloth_cube_" + str(i) + ".obj")
    convert2urdf(input_obj_name)
    input_urdf_name = input_obj_name + ".urdf"
    primitive_name = "fixed_cube.urdf"
    save_img_path = "./output/" + str(i) + ".png"
    render(input_urdf_name, save_img_path, primitive_name)