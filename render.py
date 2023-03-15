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
    p.resetDebugVisualizerCamera(2, cameraYaw = -165, cameraPitch = -30, cameraTargetPosition=[-0.5, 0.5, 0])

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
    
    # while True:
    #   pass

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


def render_clothHuman(particle_url, save_img_path, primitive_url=None):
    physicsClient = p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2, cameraYaw = 0, cameraPitch = -85, cameraTargetPosition=[0, 0, 0])

    StartPos = [0, 0, 0]
    StartOrientation = p.getQuaternionFromEuler([0, 0, 0])

    # cloth = p.loadURDF(particle_url, StartPos, StartOrientation, useFixedBase=True)
    # p.changeVisualShape(cloth, -1, rgbaColor=[67.0/255.0, 142.0/255.0, 219.0/255.0 ,1], flags=0)

    if primitive_url is not None:
      boxId = p.loadURDF(primitive_url, StartPos, StartOrientation, useFixedBase=True)
      p.changeVisualShape(boxId, -1, rgbaColor=[139.0/255.0, 71.0/255.0, 38.0/255.0 ,1], flags=0)
      states = np.loadtxt("/home/ubuntu/BulletVideo/clothHuman/p.txt")
      for index in range(p.getNumJoints(boxId)):
        p.resetJointState(boxId, index, states[index])

    p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.01)
    p.setRealTimeSimulation(0)
    embed()
    while True:
      pass

    img = p.getCameraImage(1280, 800)
    rgb_img = rgba2rgb(img[2])
    cv2.imwrite(save_img_path, rgb_img[:, :, ::-1])

    p.disconnect()