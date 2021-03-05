import os
import sys

from third_party.legacy_roslaunch import main

LAUNCH_FILES = [{launch_files}]

all_args = sys.argv + LAUNCH_FILES

main.main(all_args)
