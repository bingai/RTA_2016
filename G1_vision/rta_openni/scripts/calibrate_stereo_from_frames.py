#!/usr/bin/python3
"""
@author: Gregory Kramida
@licence: Apache v2

Copyright 2016 Gregory Kramida

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import os
import sys
import os.path
import calib.io
from calib.rig import Rig
from calib.camera import Camera
from calib.utils import calibrate, calibrate_stereo, calibrate_intrinsics
from calib.geom import generate_board_object_points
import subprocess
import cv2
from MATLAB_to_calcv_stereo import matlab_to_calcv_stereo, save_frames_with_corners
import numpy as np
import re
import argparse as ap

parser = ap.ArgumentParser("Preprocess IR range sensor frames for calibration.")
parser.add_argument("-f", "--folders", nargs="*", type=str,
                    help="folders where to retrieve images from different cameras, one for each camera",
                    default=["ir", "rgb"])
parser.add_argument("-sc", "--save_checkerboards", action='store_true', default=False,
                    help="Save frames with checkerboard overlay into separate folders")

parser.add_argument("-mc", "--MATLAB_corners", action='store_true', default=False,
                    help="Use MATLAB's corner detection (MATLAB w/ calibration toolbox must be properly " +
                         "installed on your system)")
parser.add_argument("-mci", "--MATLAB_corners_intrinsics_only", action='store_true', default=False,
                    help="Use MATLAB's corner detection, but only to calibrate intrisics " +
                         " (MATLAB w/ calibration toolbox must be properly " +
                         "installed on your system)")


def gather_image_points(directory, board_size=(15, 8)):
    files = os.listdir(directory)
    files.sort()
    frame_hash = {}
    path_hash = {}
    for file in files:
        if file.endswith(".png"):
            frame_num = int(re.search(r'\d+', file).group(0))
            path = os.path.join(directory, file)
            frame = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            success, corners = cv2.findChessboardCorners(frame, board_size)
            if success:
                cv2.cornerSubPix(frame, corners, (11, 11), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
                frame_hash[frame_num] = corners
                path_hash[frame_num] = path
    return frame_hash, path_hash


def gather_path_hash(directory, used_numbers):
    files = os.listdir(directory)
    files.sort()
    path_hash = {}
    for file in files:
        if file.endswith(".png"):
            frame_num = int(re.search(r'\d+', file).group(0))
            path = os.path.join(directory, file)
            if frame_num in used_numbers:
                path_hash[frame_num] = path
    return path_hash


def main():
    args = parser.parse_args()
    if args.MATLAB_corners_intrinsics_only and args.MATLAB_corners:
        raise ValueError(
            "Arguments -mc (--MATLAB_corners) and -mci (--MATLAB_corners_intrinsics_only) cannot be combined.")

    directory = "./"
    board_square_size = 0.05083333333
    set_one_directory = os.path.join(directory, args.folders[0])
    set_two_directory = os.path.join(directory, args.folders[1])
    postfix_one = os.path.basename(args.folders[0])
    postfix_two = os.path.basename(args.folders[1])

    test_cam1_frame = cv2.imread(os.path.join(set_one_directory, "capture_000_{:s}.png".format(postfix_one)),
                                 cv2.IMREAD_UNCHANGED)
    test_cam2_frame = cv2.imread(os.path.join(set_two_directory, "capture_000_{:s}.png".format(postfix_two)),
                                 cv2.IMREAD_UNCHANGED)

    rig = Rig(cameras=(Camera(resolution=test_cam1_frame.shape),
                       Camera(resolution=(test_cam2_frame.shape[0], test_cam2_frame.shape[1]))))

    if args.MATLAB_corners or args.MATLAB_corners_intrinsics_only:
        # run Matlab corner detection
        FNULL = open(os.devnull, 'w')
        print("Running MATLAB's corner detection...")
        subprocess.call(["matlab", "-nojvm", "-r",
                         "detectCornersStereo('" + postfix_one + "','" + postfix_two + "'); quit"], stdout=FNULL,
                        stderr=subprocess.STDOUT)
        matlab_to_calcv_stereo([set_one_directory, set_two_directory], save_images_with_corners=False)
        frame_data = np.load(os.path.join(directory, "MATLAB_aux_stereo.npz"))
        board_dims = tuple(frame_data["board_dimensions"])
        image_points_one_raw = frame_data["image_points" + postfix_one]
        image_points_two_raw = frame_data["image_points" + postfix_two]
        frame_numbers_one = frame_data["frame_numbers" + postfix_one]
        frame_numbers_two = frame_data["frame_numbers" + postfix_two]
        image_points_one = {frame_numbers_one[i_frame]: image_points_one_raw[i_frame] for i_frame in range(len(image_points_one_raw))}
        image_points_two = {frame_numbers_two[i_frame]: image_points_two_raw[i_frame] for i_frame in range(len(image_points_two_raw))}
        path_hash1 = gather_path_hash(set_one_directory, set(frame_numbers_one))
        path_hash2 = gather_path_hash(set_two_directory, set(frame_numbers_two))

    corner_folder_postfix = "_MATLAB"

    if args.MATLAB_corners_intrinsics_only:
        object_point_set = generate_board_object_points(board_dims[1], board_dims[0], board_square_size)
        print("Calibrating intrinsics using OpenCV and MATLAB corners...")
        calibrate_intrinsics(rig.cameras[0], image_points_one_raw,
                             object_point_set,
                             use_rational_model=False,
                             use_tangential=True,
                             use_thin_prism=False,
                             fix_radial=False,
                             fix_thin_prism=False,
                             max_iterations=100,
                             use_existing_guess=False,
                             test=False)
        calibrate_intrinsics(rig.cameras[1], image_points_two_raw,
                             object_point_set,
                             use_rational_model=False,
                             use_tangential=True,
                             use_thin_prism=False,
                             fix_radial=False,
                             fix_thin_prism=False,
                             max_iterations=100,
                             use_existing_guess=False,
                             test=False)
        if args.save_checkerboards:
            save_frames_with_corners(set_one_directory + "_checkerboard_matlab", list(path_hash1.values()),
                                     list(image_points_one.values()), board_dims)
            save_frames_with_corners(set_two_directory + "_checkerboard_matlab", list(path_hash2.values()),
                                     list(image_points_two.values()), board_dims)

    if not args.MATLAB_corners:
        corner_folder_postfix = "_OpenCV"
        board_dims = (15, 8)
        print("Running OpenCV's corner detection...")
        image_points_one, path_hash1 = gather_image_points(set_one_directory, board_dims)
        image_points_two, path_hash2 = gather_image_points(set_two_directory, board_dims)
        np.savez_compressed("OpenCV_aux_stereo.npz",
                            **{"image_points" + postfix_one: np.array(list(image_points_one.values())),
                               "image_points" + postfix_two: np.array(list(image_points_two.values())),
                               "frame_numbers" + postfix_one: np.array(list(image_points_one.keys())),
                               "frame_numbers" + postfix_two: np.array(list(image_points_two.keys())),
                               "board_dimensions": np.array(board_dims)})
    if args.save_checkerboards:
        save_frames_with_corners(set_one_directory + "_checkerboard" + corner_folder_postfix, list(path_hash1.values()),
                                 list(image_points_one.values()), board_dims)
        save_frames_with_corners(set_two_directory + "_checkerboard" + corner_folder_postfix, list(path_hash2.values()),
                                 list(image_points_two.values()), board_dims)

    object_point_set = generate_board_object_points(board_dims[1], board_dims[0], board_square_size)

    image_point_sets = [image_points_one, image_points_two]

    print("Calibrating stereo pair using OpenCV...")
    calibrate_stereo(rig, image_point_sets,
                     object_point_set,
                     use_fisheye=False,
                     use_rational_model=False,
                     use_tangential=True,
                     use_thin_prism=False,
                     fix_radial=False,
                     fix_thin_prism=False,
                     precalibrate_solo=True,
                     stereo_only=args.MATLAB_corners_intrinsics_only,
                     # if mode is MATLAB_corners_intrinsics_only, intrinsics have been calibrated already
                     max_iterations=100,
                     use_intrinsic_guess=False)
    print(rig)
    calib.io.save_opencv_calibration("calibration_{:s}_{:s}.xml".format(postfix_one, postfix_two), rig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
