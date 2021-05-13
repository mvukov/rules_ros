"""Implements functionality for launching ROS master and ROS nodes.
"""

load("//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

def ros_launch(name, nodes, launch_files, launch_args = None):
    """Defines a ROS deployment.

    Args:
        name: The name of the deployment.
        nodes: The nodes used by the deployment.
        launch_files: The launch files used by the deployment.
        launch_args: The launch arguments used by the deployment.
    """
    launch_file_paths = ["'$(location {})'".format(x) for x in launch_files]
    launch_args = launch_args or []
    substitutions = {
        "{launch_files}": ", ".join(launch_file_paths),
        "{launch_args}": ", ".join(launch_args),
    }

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "@com_github_mvukov_rules_ros//ros:launch.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = launch_files,
    )

    py_binary(
        name = name,
        srcs = [launch_script],
        data = nodes + launch_files,
        main = launch_script,
        visibility = ["//visibility:public"],
        deps = ["@com_github_mvukov_rules_ros//third_party/legacy_roslaunch"],
    )
