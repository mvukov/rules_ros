import sys

from third_party.legacy_roslaunch import main as roslaunch_main

LAUNCH_FILES = [{launch_files}]

all_args = sys.argv[:1] + LAUNCH_FILES + sys.argv[1:]

roslaunch_main.main(all_args)
