import os
from mesh2urdf import convert2urdf
from render import render_clothHanger, render_clothFold, render_clothWash, render_clothHuman

exp_name = ['clothHanger', 'clothFold', 'clothWash', 'clothHuman']
num = [201, 60, 200, 3000]
j=3

for i in range(1750, num[j], 5):
    input_obj_name = os.path.join("./{}/cloth".format(exp_name[j]), str(i))
    # convert2urdf(input_obj_name)
    input_urdf_name = input_obj_name + ".urdf"
    primitive_name = "./{}/human2.urdf".format(exp_name[j])
    save_img_path = "./{}/output/".format(exp_name[j]) + str(i//5) + ".png"
    render_clothHuman(input_urdf_name, save_img_path, i)