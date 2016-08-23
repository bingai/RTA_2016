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
import re
import shutil

parser = ap.ArgumentParser("Add a constant to all frame numbers in current folder.")
parser.add_argument("-i", "--increment", type=int, help="Constant to increment the frame number by.")


def main():
    args = parser.parse_args()
    directory = "./"
    all_files = os.listdir(directory)
    num_pattern = re.compile(r"\d\d\d")
    all_files.sort()
    for file in all_files:
        sr = num_pattern.search(file)
        path = os.path.join(directory, file)
        if os.path.isfile(path) and sr is not None:
            number = int(sr.group(0))
            new_name = file.replace(str(sr.group(0)), "{:03d}".format(number + args.increment))
            new_path = os.path.join(directory, new_name)
            if os.path.exists(new_path):
                print("File exists: {0:s}".format(new_path))
                return 1
            else:
                shutil.move(path, new_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
