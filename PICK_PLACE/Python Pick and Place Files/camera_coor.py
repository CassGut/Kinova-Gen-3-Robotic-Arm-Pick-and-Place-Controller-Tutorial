import multiprocessing
import subprocess
import cv2
from scipy.optimize import linprog_verbose_callback
from alpha import alpha
import cv2.aruco as aruco
from smallest_angular_distance import smallest_angular_distance_limits
from smallest_angular_distance import smallest_angular_distance_nolimits
from angle_guess import angle_guess
from trust_constr import trust_constr 
from Rx import Rx
from Ry import Ry
from Rz import Rz
from T import T
from SM import SM
from CM import CM
from NR import NR
from WM import WM
from GN import GN
from fk import fk
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2
import sys
import os
import time
import threading
import numpy as np
from close_gripper import close_gripper
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from get_angles_pose import get_angles_pose
from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.messages import Session_pb2, VisionConfig_pb2

def camera_coor(pose):
    x = pose.x
    y =pose.y
    z =pose.z
    X = pose.theta_x
    Y = pose.theta_y
    Z = pose.theta_z
    rx = Rx(np.deg2rad(X))
    ry = Ry(np.deg2rad(Y))
    rz = Rz(np.deg2rad(Z))
    terms1 = [rz,ry,rx]
    R = np.linalg.multi_dot(terms1)
    M = np.eye(4)
    M[:3, :3] = R[:3, :3]
    M[:3, 3] = [x, y, z]
    terms2= [M,T(0, 0.054, -0.138)]
    # terms2= [M,T(0, 0.054, -0.124)]
    C = np.linalg.multi_dot(terms2)
    return C

def main():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

    import utilities
    
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router, utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        angles = base.GetMeasuredJointAngles()
        print(angles)

if __name__ == "__main__":
    main()
    input()
