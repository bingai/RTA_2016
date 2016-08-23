#!/usr/bin/python3

import numpy as np
import sys
import os
import os.path
import cv2
import re


def save_frames_with_corners(chessboard_folder, paths, corners, board_dims):
    if len(paths) != len(corners):
        raise ValueError("Different number of images and corners. Aborting.")
    if not os.path.exists(chessboard_folder):
        os.makedirs(chessboard_folder)
    ix_chessboard = 0
    for path in paths:
        image = cv2.imread(path)
        image = cv2.drawChessboardCorners(image, board_dims,
                                          corners[ix_chessboard].astype(np.float32), True)
        cv2.imwrite(os.path.join(chessboard_folder, os.path.basename(path)), image)
        ix_chessboard += 1


def matlab_to_calcv_stereo(frame_folders, camera_names=None, save_images_with_corners=False, cleanup_after_MATLAB=True):
    if camera_names is None:
        camera_names = [os.path.basename(frame_folder) for frame_folder in frame_folders]

    board_size_filename = "boardSize_" + "stereo" + ".csv"
    # point array should be:
    # [number of images] X [number of points in long side] X [number of points in short side] X [2 coordinates]
    board_dims = tuple(np.genfromtxt(board_size_filename, delimiter=',').astype(np.int32) - 1)

    archive = {}

    for ix_cam in range(0, len(frame_folders)):
        frame_folder = frame_folders[ix_cam]
        frame_folder_base = os.path.basename(frame_folder)
        frames_used_filename = "imagesUsed_" + frame_folder_base + ".csv"
        used_frame_numbers = np.genfromtxt(frames_used_filename, delimiter=',').astype(np.int32)
        camera_name = camera_names[ix_cam]
        points_filename = "imagePoints_" + frame_folder_base + ".csv"
        points = np.genfromtxt(points_filename, delimiter=',')
        points.resize(board_dims[1], board_dims[0], points.shape[1] // 2, 2)
        points = np.transpose(points, (2, 1, 0, 3))
        # Back to opencv format
        points = points.reshape(points.shape[0], points.shape[1] * points.shape[2], 1, points.shape[3])
        for i_pt_set in range(len(points)):
            pt_set = points[i_pt_set]
            # first point should always have a greater y coordinate
            if (pt_set[0][0][1] < pt_set[-1][0][1]):
                # if id doesn't reverse the points' order
                points[i_pt_set] = np.flipud(pt_set)

        # if not os.path.isfile(video_path) or video_path[-4:] != ".mp4":
        #     raise ValueError("Could not find a valid mp4 video file at {:s}".format(video_path))
        archive["image_points" + camera_name] = points.astype(np.float32)
        archive["frame_numbers" + camera_name] = used_frame_numbers

        if save_images_with_corners:
            used_frame_numbers = []

            frame_filenames = os.listdir(frame_folder)
            frame_filenames.sort()
            filename_by_number = {}
            for filename in frame_filenames:
                if os.path.isfile(os.path.join(frame_folder, filename)) and filename.endswith(".png"):
                    frame_num = int(re.search(r'\d+', filename).group(0))
                    filename_by_number[frame_num] = filename

            used_paths = [os.path.join(frame_folder, filename_by_number[frame_number]) for frame_number in
                          used_frame_numbers]

            chessboard_folder = frame_folder + "_MATLAB_chessboard"
            save_frames_with_corners(chessboard_folder, used_paths, points, (board_dims[1], board_dims[0]))
        if cleanup_after_MATLAB:
            os.remove(frames_used_filename)
            os.remove(points_filename)

    archive["board_dimensions"] = np.array([board_dims[1], board_dims[0]])
    np.savez_compressed("MATLAB_aux_" + "stereo", **archive)
    if cleanup_after_MATLAB:
        os.remove(board_size_filename)

    return archive


def main():
    if len(sys.argv) < 2 or sys.argv[1] == '-h' or sys.argv[1] == '--help' or len(sys.argv) < 5:
        print(
            "Usage: ./MATLAB_to_calcv.py frame_folder1 video_path1 frame_folder2 video_path2 [--save_chessboard_images|-s]")
        sys.exit(1)
    frame_folder1 = sys.argv[1]
    video_path1 = sys.argv[2]
    frame_folder2 = sys.argv[3]
    video_path2 = sys.argv[4]

    if len(sys.argv) > 5:
        arg = sys.argv[5]
        # optionally, draw the detected chessboard points on top of the source images and save to a separate folder
        if arg in ["--save_chessboard_images", "-s"]:
            save_images_with_corners = True

    frame_folders = [frame_folder1, frame_folder2]
    video_paths = [video_path1, video_path2]
    camera_names = [os.path.basename(video_path)[:-4] for video_path in video_paths]
    matlab_to_calcv_stereo(frame_folders, camera_names, save_images_with_corners)

    return 0


if __name__ == '__main__':
    sys.exit(main())
