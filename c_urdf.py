# generate a single body urdf, given the object name
# ensure the .obj file is obj_name.obj

import os
import glob
import math
import numpy as np
import sys
import xml.etree.ElementTree as ET
from xml.etree.ElementTree import Element, tostring, SubElement, Comment, ElementTree, XML
# import network.transformations as transformations
# from scipy.spatial.transform import Rotation as R

def set_fix_joint(child_id, parent_id):
    joint_element = Element('joint', {'name': "joint_" + str(child_id), "type": "fixed"})
    origin = Element('origin', {'xyz': "0 0 0"})
    joint_element.append(origin)
    child = Element('child', {'link': "link_" + str(child_id)})
    joint_element.append(child)
    if parent_id == -1:
        parent = Element('parent', {'link': "world"})
    else:
        parent = Element('parent', {'link': "link_" + str(parent_id)})
    joint_element.append(parent)
    return joint_element

def set_rev_joint(child_id, parent_id, pos, ori):
    joint_element = Element('joint', {'name': "joint_" + str(child_id), "type": "revolute"})
    origin = Element('origin', {'xyz': "%6f %6f %6f" % (pos[0], pos[1], pos[2]), "rpy": "%6f %6f %6f" % (ori[0], ori[1], ori[2])})
    joint_element.append(origin)
    axis = Element('axis', {'xyz': "0 0 1"})
    joint_element.append(axis)
    child = Element('child', {'link': "link_" + str(child_id)})
    joint_element.append(child)
    parent = Element('parent', {'link': "link_" + str(parent_id)})
    joint_element.append(parent)
    limit = Element('limit', {'lower': "-3.14", "upper": "3.14", "effort": "30", "velocity": "10"})
    joint_element.append(limit)
    dynamics = Element('dynamics', {'damping': '0.0', 'friction': '0.0'})
    joint_element.append(dynamics)
    return joint_element

def set_pri_joint(child_id, parent_id, pos, ori):
    joint_element = Element('joint', {'name': "joint_" + str(child_id), "type": "prismatic"})
    origin = Element('origin', {'xyz': "%6f %6f %6f" % (pos[0], pos[1], pos[2]), "rpy": "%6f %6f %6f" % (ori[0], ori[1], ori[2])})
    joint_element.append(origin)
    axis = Element('axis', {'xyz': "0 0 1"})
    joint_element.append(axis)
    child = Element('child', {'link': "link_" + str(child_id)})
    joint_element.append(child)
    parent = Element('parent', {'link': "link_" + str(parent_id)})
    joint_element.append(parent)
    limit = Element('limit', {'lower': "-1", "upper": "1", "effort": "30", "velocity": "10"})
    joint_element.append(limit)
    dynamics = Element('dynamics', {'damping': '0.0', 'friction': '0.0'})
    joint_element.append(dynamics)
    return joint_element

def set_link(link_id, pos=[0,0,0], obj_name=""):
    link_element = Element('link', {'name': "link_" + str(link_id)})
    link_origin = Element('origin', {'xyz': "%6f %6f %6f" % (pos[0], pos[1], pos[2]), "rpy": "0 0 0"})
    inertial = Element('inertial')
    inertial_mass = Element('mass', {'value': '1.0'})
    inertial.append(inertial_mass)
    inertial.append(link_origin)
    inertial_inertia = Element('inertia', {'ixx': '1.0', 'ixy': '0.0', 'ixz': '0.0', 'iyy': '1.0', 'iyz': '0.0', 'izz': '1.0'})
    inertial.append(inertial_inertia)
    link_element.append(inertial)
    visual = Element('visual')
    visual.append(link_origin)
    visual_geometry = Element('geometry')
    # visual_geometry_mesh = Element('mesh', {'filename': str(link_id) + "_manifold.obj"})
    visual_geometry_mesh = Element('mesh', {'filename': obj_name + '.obj'})
    visual_geometry.append(visual_geometry_mesh)
    visual.append(visual_geometry)
    # visual_material = Element('material', {'name': "white"})
    # visual_material_color = Element('color', {'rgba': "1 1 1 1"})
    # visual_material.append(visual_material_color)
    # visual.append(visual_material)
    link_element.append(visual)
    collision = Element('collision')
    collision.append(link_origin)
    collision_geometry = Element('geometry')
    # collision_geometry_mesh = Element('mesh', {'filename': str(link_id) + "_manifold.obj"})
    collision_geometry_mesh = Element('mesh', {'filename': obj_name + '.obj'})
    collision_geometry.append(collision_geometry_mesh)
    collision.append(collision_geometry)
    link_element.append(collision)
    return link_element

def get_joint_info(joint_info):
    joint_pos = joint_info[:3]
    joint_ori = joint_info[3:] - joint_info[:3]

    axis = joint_ori
    z = np.array([0, 0, 1])
    axis = axis / np.linalg.norm(axis)
    z = z / np.linalg.norm(z)

    sita = math.acos(axis @ z)

    n_vector = np.cross(axis, z) 
    n_vector = n_vector / np.linalg.norm(n_vector)

    n_vector_invert = np.array([[0,-n_vector[2],n_vector[1]],
                                [n_vector[2],0,-n_vector[0]],
                                [-n_vector[1],n_vector[0],0]
                                ])

    I = np.array([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
                ])

    R = I + math.sin(sita)*n_vector_invert + n_vector_invert@(n_vector_invert)*(1-math.cos(sita))

    R = R.T
    # joint_ori = transformations.euler_from_matrix(R)
    joint_ori = None
    assert joint_ori is not None, 'should not reach here'

    return joint_pos, joint_ori, R

def single_body_urdf(obj_name):
    tree = ET.parse("./init/init.urdf")
    root = tree.getroot()
    base_link = set_link(int(0), obj_name=obj_name)
    root.append(base_link)
    # if not os.path.exists(path + "mesh"):
    #      os.makedirs(path + "mesh")
    obj_urdf = obj_name + '.urdf'
    tree.write(obj_urdf, encoding='utf-8', xml_declaration=True)


def one_sample(path):
    mask = np.load(path + "mask.npy")
    link_map = np.load(path + "link_map.npy")
    #print(link_map)
    new_link_map = np.zeros(link_map.shape)
    count = 1
    id_mask = np.zeros((np.unique(mask).max(),))
    for i in range(link_map.shape[0]):
        id_mask[int(link_map[i,0]) - 1] = 1
        if link_map[i,1] == -1:
            base_id = link_map[i,0]
            new_link_map[0] = link_map[i]
    id_mask[int(base_id) - 1] = 0
    tree = ET.parse("./init.urdf")
    root = tree.getroot()
    base_link = set_link(int(base_id))
    root.append(base_link)
    base_joint = set_fix_joint(int(base_id), -1)
    root.append(base_joint)
    part_R = np.zeros((np.unique(mask).max(),4,4))
    part_R[:] = np.eye(4)

    while (id_mask == 1).sum() != 0:
        for i in range(link_map.shape[0]):
            link_info = link_map[i]
            parent_id = link_info[1]
            child_id = link_info[0]
            if id_mask[int(child_id) - 1] == 0:
                continue
            if id_mask[int(parent_id) - 1] != 0:
                continue
            link = set_link(int(child_id))
            root.append(link)
            id_mask[int(child_id) - 1] = 0
            if link_info[3] == 1:
                joint_pos, joint_ori, R = get_joint_info(link_info[4:10])
                joint = set_rev_joint(int(child_id), int(parent_id), joint_pos, joint_ori)
                root.append(joint)
                part_R[int(child_id) - 1,:3,:3] = R
                part_R[int(child_id) - 1,:3,3] = joint_pos
            elif link_info[3] == 2:
                joint_pos, joint_ori, R = get_joint_info(link_info[4:10])
                joint = set_pri_joint(int(child_id), int(parent_id), joint_pos, joint_ori)
                root.append(joint)
                part_R[int(child_id) - 1,:3,:3] = R
                part_R[int(child_id) - 1,:3,3] = joint_pos
            elif link_info[3] == 3:
                joint = set_fix_joint(int(child_id), int(parent_id))
                root.append(joint)
            new_link_map[count] = link_info
            count = count + 1

    np.save(path + "R.npy", part_R)
    print(new_link_map[:,:5])
    np.save(path + "link_map.npy", new_link_map)
    if not os.path.exists(path + "mesh"):
         os.makedirs(path + "mesh")
    tree.write(path + "mesh/model.urdf", encoding='utf-8', xml_declaration=True)

# if __name__ == "__main__":
#     path_list = glob.glob("./out/final_test_pose2/*/*/")
#     for path in path_list:
#         print(path)
#         one_sample(path)

def convert2urdf(obj_name):
    single_body_urdf(obj_name)
