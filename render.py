import pybullet as p
from time import sleep
import pybullet_data
import cv2
import numpy as np
from IPython import embed

def rgba2rgb(rgba, background=(255,255,255)):
    row, col, ch = rgba.shape

    if ch == 3:
        return rgba
    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros( (row, col, 3), dtype='float32' )
    r, g, b, a = rgba[:,:,0], rgba[:,:,1], rgba[:,:,2], rgba[:,:,3]
    a = np.asarray( a, dtype='float32' ) / 255.0
    R, G, B = background

    rgb[:,:,0] = r * a + (1.0 - a) * R
    rgb[:,:,1] = g * a + (1.0 - a) * G
    rgb[:,:,2] = b * a + (1.0 - a) * B

    return np.asarray( rgb, dtype='uint8' )



def render_clothHanger(particle_url, save_img_path, primitive_url=None):
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(1, cameraYaw = -165, cameraPitch = -30, cameraTargetPosition=[-0.5, 1, 0.2])

    StartPos = [0, 0, 0]
    StartOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])

    cloth = p.loadURDF(particle_url, StartPos, StartOrientation, useFixedBase=True)
    p.changeVisualShape(cloth, -1, rgbaColor=[67.0/255.0, 142.0/255.0, 219.0/255.0 ,1], flags=0)

    if primitive_url is not None:
      StartPos = [0, 1, 0.05]
      boxId = p.loadURDF(primitive_url, StartPos, StartOrientation, useFixedBase=True)
      p.changeVisualShape(boxId, -1, rgbaColor=[139.0/255.0, 71.0/255.0, 38.0/255.0 ,1], flags=0)

    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.01)
    p.setRealTimeSimulation(0)
    
    while True:
      pass

    img = p.getCameraImage(1280, 800)
    rgb_img = rgba2rgb(img[2])
    cv2.imwrite(save_img_path, rgb_img[:, :, ::-1])

    p.disconnect()



def render_clothFold(particle_url, save_img_path, primitive_url=None):
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2, cameraYaw = 0, cameraPitch = -85, cameraTargetPosition=[0, 0, 0])

    StartPos = [0, 0, 1]
    StartOrientation = p.getQuaternionFromEuler([0, 0, 0])

    cloth = p.loadURDF(particle_url, StartPos, StartOrientation, useFixedBase=True)
    p.changeVisualShape(cloth, -1, rgbaColor=[67.0/255.0, 142.0/255.0, 219.0/255.0 ,1], flags=0)

    if primitive_url is not None:
      boxId = p.loadURDF(primitive_url, StartPos, StartOrientation, useFixedBase=True)
      p.changeVisualShape(boxId, -1, rgbaColor=[139.0/255.0, 71.0/255.0, 38.0/255.0 ,1], flags=0)

    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.01)
    p.setRealTimeSimulation(0)

    # while True:
    #   pass

    img = p.getCameraImage(1280, 800)
    rgb_img = rgba2rgb(img[2])
    cv2.imwrite(save_img_path, rgb_img[:, :, ::-1])

    p.disconnect()


def render_clothWash(particle_url, save_img_path, primitive_url=None):
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2, cameraYaw = 0, cameraPitch = -85, cameraTargetPosition=[0, 0, 0])

    StartPos = [0, 0, 0]
    StartOrientation = p.getQuaternionFromEuler([0, 0, 0])

    cloth = p.loadURDF(particle_url, StartPos, StartOrientation, useFixedBase=True)
    p.changeVisualShape(cloth, -1, rgbaColor=[67.0/255.0, 142.0/255.0, 219.0/255.0 ,1], flags=0)

    if primitive_url is not None:
      boxId = p.loadURDF(primitive_url, StartPos, StartOrientation, useFixedBase=True)
      p.changeVisualShape(boxId, -1, rgbaColor=[139.0/255.0, 71.0/255.0, 38.0/255.0 ,1], flags=0)

    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.01)
    p.setRealTimeSimulation(0)

    while True:
      pass

    img = p.getCameraImage(1280, 800)
    rgb_img = rgba2rgb(img[2])
    cv2.imwrite(save_img_path, rgb_img[:, :, ::-1])

    p.disconnect()


def render_clothHuman(particle_url, save_img_path, step):
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(8, cameraYaw = 90, cameraPitch = -30, cameraTargetPosition=[0, 0, 0])

    StartPos = [0, 0, 0]
    StartOrientation = p.getQuaternionFromEuler([np.pi/2, 0, 0])

    cloth = p.loadURDF(particle_url, StartPos, StartOrientation, useFixedBase=True)
    p.changeVisualShape(cloth, -1, rgbaColor=[67.0/255.0, 142.0/255.0, 219.0/255.0 ,1], flags=0)

    if step < 10:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos0.txt")
    elif step >= 10 and step < 20:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos1_12.txt")
    elif step >= 20 and step < 30:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos2_12.txt")
    elif step >= 30 and step < 40:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos3_12.txt")
    elif step >= 40 and step < 50:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos4_12.txt")
    elif step >= 50 and step < 60:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos5_12.txt")
    elif step >= 60 and step < 1700:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos6_12.txt")


    elif step >= 1700 and step < 1710:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos7_12.txt")
    elif step >= 1710 and step < 1720:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos8_12.txt")
    elif step >= 1720 and step < 1730:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos9_12.txt")
    elif step >= 1730 and step < 1740:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos10_12.txt")
    elif step >= 1740 and step < 1750:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos11_12.txt")
    elif step >= 1750 and step < 2800:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos11_12.txt")
    

    elif step >= 2800 and step < 2810:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos12_12.txt")
    elif step >= 2810 and step < 2820:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos10_12.txt")
    elif step >= 2820 and step < 2830:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos9_12.txt")
    elif step >= 2830 and step < 2840:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos8_12.txt")
    elif step >= 2840 and step < 2850:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos7_12.txt")
    elif step >= 2850 and step < 2860:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos6_12.txt")
    elif step >= 2860 and step < 2870:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos5_12.txt")
    elif step >= 2870 and step < 2880:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos4_12.txt")
    elif step >= 2880 and step < 2890:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos3_12.txt")
    elif step >= 2890 and step < 2900:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos2_12.txt")
    elif step >= 2900 and step < 2910:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos1_12.txt")
    elif step >= 2910 and step < 3000:
        pos_vel = np.loadtxt("./clothHuman/human_pos_info/pos0.txt")

    robot_part = ['sphere_0.36','sphere_0.52','capsule_0.72','capsule_0.54',
     'sphere_0.16','sphere_0.41','capsule_0.72','capsule_0.54',
     'sphere_0.16','capsule_1.2','capsule_1.24','box','capsule_1.2','capsule_1.24','box']

    for i in range(15):
      primitive_url = './clothHuman/mesh/' + robot_part[i] + '.urdf'
      StartPos = pos_vel[i*6+3 : i*6+6]
      StartPos = np.array([StartPos[0], -StartPos[2], StartPos[1]])
      StartOrientation = p.getQuaternionFromEuler(pos_vel[i*6 : i*6+3] + np.array([np.pi/2,0,0]))
      boxId = p.loadURDF(primitive_url, StartPos, StartOrientation, useFixedBase=True)
      p.changeVisualShape(boxId, -1, rgbaColor=[139.0/255.0, 71.0/255.0, 38.0/255.0 ,1], flags=0)

    # p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.01)
    # p.setRealTimeSimulation(0)

    # while True:
    #   pass

    img = p.getCameraImage(1280, 800)
    rgb_img = rgba2rgb(img[2])
    cv2.imwrite(save_img_path, rgb_img[:, :, ::-1])

    p.disconnect()