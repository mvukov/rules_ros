"""Implements functionality for launching ROS master and ROS nodes.
"""

load("//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

ROSCORE_XML = "//third_party/legacy_roslaunch:roscore.xml"

def ros_launch(name, nodes, launch_files):
    launch_script = "{}_launch.py".format(name)

    all_launch_targets = [ROSCORE_XML] + launch_files
    all_launch_file_paths = ["'$(location {})'".format(x) for x in all_launch_targets]

    substitutions = {
        "{launch_files}": ", ".join(all_launch_file_paths),
    }

    expand_template(
        name = "{}_launch_gen".format(name),
        template = "//ros:launch.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = all_launch_targets,
    )

    py_binary(
        name = name,
        srcs = [launch_script],
        data = [
            ROSCORE_XML,
            "@ros_comm//:rosmaster",
            "@ros_comm//:rosout",
        ] + nodes + launch_files,
        main = launch_script,
        visibility = ["//visibility:public"],
        deps = ["//third_party/legacy_roslaunch"],
    )
