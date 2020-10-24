import os
import cv2
import PIL
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.spatial.transform import Rotation
from camera_handler import CameraHandler

#ee: end efector

# camera 1
CAMERA_INTRINSICS_MAT =  np.array([np.array([606.6464233398438,    0.0,                    639.0460205078125]),
                                np.array([0.0,                  606.6519775390625,      368.244140625]),
                                np.array([0.0,                  0.0,                    1.0])])
CAMERA_DISTORTION_COEFF_MAT = np.array([0.5164358615875244,     -2.606694221496582,     0.00045736812171526253,     -0.00019684531434904784,
                                    1.499117374420166,      0.39795395731925964,    -2.4385111331939697,        1.4303737878799438])
ARUCO_NAME = cv2.aruco.DICT_5X5_100
MARKER_SIDE_LENGTH_MM = 0.0996

class HandEyeCalibration():
    def __init__(self, camera_intrinsics_mat, camera_distortion_coeff_mat):
        self.cam_intr_mat = camera_intrinsics_mat
        self.cam_dist_coeff_mat = camera_distortion_coeff_mat
        self.marker_side_length_mm = MARKER_SIDE_LENGTH_MM
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_NAME)
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.marker_to_cam_poses_list = []
        self.R_marker_to_cam_poses_list = []
        self.t_marker_to_cam_poses_list = []
        self.ee_to_base_poses_list = []
        self.R_ee_to_base_poses_list = []
        self.t_ee_to_base_poses_list = []
        self.realsense_handler = CameraHandler((640, 480))
    def draw_marker(self):
        img = cv2.aruco.drawMarker(self.aruco_dict, 0, 700)
        plt.imshow(img, cmap=mpl.cm.gray)
        plt.show()

if __name__ == '__main__':  
    calib = HandEyeCalibration(
        CAMERA_INTRINSICS_MAT, CAMERA_DISTORTION_COEFF_MAT)

    # Get R,t ee_to_base 
    calib.read_ee_to_base_poses_from_file("/home/robopc/repos/sai2_environment/poseright/pose.txt")
    T_ee_base = calib.ee_to_base_poses_list
    R_ee_base = calib.R_ee_to_base_poses_list
    t_ee_base = calib.t_ee_to_base_poses_list
    
    # Get R,T c_to_marker
    img_list = calib.get_images_list_from_dir("/home/robopc/repos/sai2_environment/imageright")
    calib.get_poses_from_images(img_list)
    R_marker_cam = calib.R_marker_to_cam_poses_list
    t_marker_cam = calib.t_marker_to_cam_poses_list
    R_cam_marker = []
    t_cam_marker = []
    
    for r,t in zip(R_marker_cam,t_marker_cam):
        R_cam_marker.append(r.T) 
        t_cam_marker.append(-r.T.dot(t))

    R_marker_to_ee, t_marker_to_ee = cv2.calibrateHandEye(R_ee_base,t_ee_base,
                                                        R_cam_marker,
                                                        t_cam_marker,
                                                        cv2.CALIB_HAND_EYE_DANIILIDIS)

    # T_cam_marker
    T_cam_marker = np.concatenate(
            [R_cam_marker[0], t_cam_marker[0].reshape((3, 1))], axis=1)
    T_cam_marker = np.concatenate(
            [T_cam_marker, np.array([0, 0, 0, 1]).reshape((1, 4))], axis=0)

    # T_marker_to_ee
    T_marker_to_ee = np.concatenate(
            [R_marker_to_ee, t_marker_to_ee.reshape((3, 1))], axis=1)
    T_marker_to_ee = np.concatenate(
            [T_marker_to_ee, np.array([0, 0, 0, 1]).reshape((1, 4))], axis=0)
    
    T_cam_base = T_ee_base[0].dot(T_marker_to_ee).dot(T_cam_marker)
    print(T_cam_base)
