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
import sys
import argparse as ap
import os
import os.path
import shutil
import cv2

parser = ap.ArgumentParser("Convert jpg to png... Runs on all images in current folder")

def main():
    parser.parse_args()
    directory = "./"
    all_files = os.listdir(directory)
    all_files.sort()
    for file in all_files:
        path = os.path.join(directory, file)
        if os.path.isfile(path):
            im = cv2.imread(path)
            cv2.imwrite(path.replace("jpg","png"), im)
    return 0


if __name__ == "__main__":
    sys.exit(main())
