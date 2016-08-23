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
import os.path
import sys
import re
import cv2
import shutil
import numpy as np
import argparse as ap

parser = ap.ArgumentParser("Preprocess IR range sensor frames for calibration.")
parser.add_argument("-ri", "--reindex", action='store_true', default=False,
                    help="Renumber frames to start at 000 and go up in consecutive order." +
                         " Warning: only paired IR & RGB frames will be preserved.")
parser.add_argument("-irc", "--ir_stretch_contrast", action='store_true', default=False,
                    help="Stretch contrast for IR frames.")

parser.add_argument("-dc", "--depth_stretch_contrast", action='store_true', default=False,
                    help="Stretch contrast for depth frames.")

parser.add_argument("-sf", "--split_to_folders", action='store_true', default=False,
                    help="Sort frames into separate folders, e.g. 'rgb', 'ir'.")

parser.add_argument("-f", "--folders", type=str, nargs="*", default=["."],
                    help="Set folders to combine images from & reindex.")

parser.add_argument("-px", "--frame_set_postfixes", nargs="*", type=str,
                    help="file postfixes for frame sets, e.g. 'arm.jpg' or 'depth.png'", default=["ir", "rgb"])


def save_frame_set(frames, paths):
    ix_frame = 0
    for path in paths:
        cv2.imwrite(path, frames[ix_frame])
        ix_frame += 1


def backup_frames(directory, paths):
    backup_dir = os.path.join(directory, "backup")
    if not os.path.exists(backup_dir):
        os.makedirs(backup_dir)
    for path in paths:
        base_fn = os.path.basename(path)
        new_path = os.path.join(backup_dir, base_fn)
        shutil.move(path, new_path)


def read_frames(directory, postfix, ext="png", pattern=r"\d\d\d"):
    paths = []
    frames = []
    filenames = os.listdir(directory)
    filenames.sort()
    numbers = []
    for fn in filenames:
        full_file_path = os.path.join(directory, fn)
        if os.path.isfile(full_file_path):
            if fn.endswith(postfix + "." + ext):
                paths.append(full_file_path)
                frames.append(cv2.imread(full_file_path, cv2.IMREAD_UNCHANGED))
                numbers.append(int(pattern.search(fn).group(0)))
    return frames, paths, np.array(numbers)


def stretch_16U_contrast(all_frames, postfix):
    processed_frames = []
    for frame in all_frames[postfix]:
        processed_frames.append(
            cv2.normalize(frame, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX))
    all_frames[postfix] = processed_frames


def main():
    args = parser.parse_args()

    all_frames = {}
    all_frame_hashes = {}
    all_paths = {}
    extensions = {}
    postfixes = []
    fn_pat = re.compile(r"\d\d\d")

    for postfix_combo in args.frame_set_postfixes:
        split = postfix_combo.split(".")
        postfix = split[0]
        ext = "png" if len(split) == 1 else split[1]
        extensions[postfix] = ext
        all_frames[postfix] = []
        all_frame_hashes[postfix] = {}
        all_paths[postfix] = []
        postfixes.append(postfix)

    max_number = -1
    for directory in args.folders:
        max_in_folder = max_number
        min_in_folder = sys.maxsize
        number_sets = {}
        frames_in_folder = {}
        paths_in_folder = {}
        for postfix in postfixes:
            frames_in_folder[postfix], paths_in_folder[postfix], numbers = read_frames(directory, postfix,
                                                                                       extensions[postfix], fn_pat)
            number_sets[postfix] = numbers
            max_in_folder = numbers.max() if numbers.max() > max_in_folder else max_in_folder
            min_in_folder = numbers.min() if numbers.min() < min_in_folder else min_in_folder
        diff = 0
        if min_in_folder <= max_number:
            diff = max_number - min_in_folder
            for postfix in postfixes:
                number_sets[postfix] += diff
        max_number = max_in_folder + diff

        for postfix in postfixes:
            all_frame_hashes[postfix].update(
                {number_sets[postfix][i]: frames_in_folder[postfix][i] for i in range(len(number_sets[postfix]))})
            all_frames[postfix] += frames_in_folder[postfix]
            all_paths[postfix] += paths_in_folder[postfix]

    if args.reindex:
        all_frames_by_number_sets = []
        for postfix in all_frames.keys():
            frame_number_set = set(all_frame_hashes[postfix].keys())
            all_frames_by_number_sets.append(frame_number_set)

        kept_frame_numbers = list(set.intersection(*all_frames_by_number_sets))
        kept_frame_numbers.sort()
        for postfix in all_frames.keys():
            frames_by_number = all_frame_hashes[postfix]
            new_frames = []
            new_paths = []
            ix_frame = 0
            for frame_number in kept_frame_numbers:
                new_frames.append(frames_by_number[frame_number])
                new_paths.append(os.path.join(".", "capture_{:03d}_{:s}.{:s}".format(ix_frame, postfix, "png")))
                ix_frame += 1
            all_frames[postfix] = new_frames
            all_paths[postfix] = new_paths

    if args.ir_stretch_contrast and "ir" in postfixes:
        stretch_16U_contrast(all_frames, "ir")

    if args.depth_stretch_contrast and "depth" in postfixes:
        stretch_16U_contrast(all_frames, "depth")

    if args.split_to_folders:
        for postfix in postfixes:
            new_dir = os.path.join("./", postfix)
            if not os.path.exists(new_dir):
                os.makedirs(new_dir)
            all_paths[postfix] = [os.path.join(new_dir, os.path.basename(path)) for path in all_paths[postfix]]

    # save
    for postfix in all_frames.keys():
        save_frame_set(all_frames[postfix], all_paths[postfix])

    return 0


if __name__ == "__main__":
    sys.exit(main())
