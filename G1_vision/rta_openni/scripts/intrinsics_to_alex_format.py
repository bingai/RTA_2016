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
from calib import io, xml

import os
import os.path
import sys
from lxml import etree

import argparse as ap

parser = ap.ArgumentParser("Extract camera intrinsics and convert to Alex's own format")
parser.add_argument("-f", "--file", help="Path to calibration data file containing data for a whole rig.", type=str,
                    default="calibration.xml")
parser.add_argument("-cn", "--camera_name", help="Name of the camera (per Alex's format).", type=str,
                    default="cam")
parser.add_argument("-ci", "--camera_index", help="Index of the camera in the calibration whose intrinsics to extract.",
                    type=int, default=0)
parser.add_argument("-o", "--output", help="Path to file where to write the intrinsics", type=str,
                    default="alex_intrinsics.xml")


def main():
    args = parser.parse_args()
    if not os.path.exists(args.file):
        print("Could not find calibration data file at {:s}".format(args.file))
        return 1
    rig = io.load_opencv_calibration(args.file)
    if args.camera_index > len(rig.cameras):
        print("Camera index given ({:d}) is too high. Total number of cameras in provided file is {:d}, " +
              "and the index has to be 0-based.".format(args.camera_index, len(rig.cameras)))
    intrinsics = rig.cameras[args.camera_index].intrinsics

    def generate_xml(root):
        xml.make_opencv_matrix_xml_element(root, intrinsics.intrinsic_mat, "K_" + args.camera_name)
        xml.make_opencv_matrix_xml_element(root, intrinsics.distortion_coeffs, "d_" + args.camera_name)
        xml.make_opencv_size_xml_element(root, (intrinsics.resolution[1], intrinsics.resolution[0]),
                                         "size_" + args.camera_name)

    io.save_opencv_xml_file(args.output, generate_xml)

    return 0


if __name__ == "__main__":
    sys.exit(main())
