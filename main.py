import os
from mesh2urdf import convert2urdf
from render import render_clothHanger, render_clothFold, render_clothWash, render_clothHuman, render_gripper

exp_name = ['clothHanger', 'clothFold', 'clothWash', 'clothHuman', 'exp-c']
num = [201, 60, 200, 3000, 200]
j=4

for i in range(0, num[j]):
    input_obj_name = os.path.join("./{}/cloth".format(exp_name[j]), str(i))
    # convert2urdf(input_obj_name)
    input_urdf_name = input_obj_name + ".urdf"
    primitive_name = "./{}/SimpleHand/gripper.urdf".format(exp_name[j])
    save_img_path = "./{}/output/".format(exp_name[j]) + str(i) + ".png"
    render_gripper(input_urdf_name, save_img_path, i, primitive_name)