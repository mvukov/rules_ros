"""Implements functionality for launching ROS master and ROS nodes.
"""

load("//third_party:expand_template.bzl", "expand_template")
load("@rules_python//python:defs.bzl", "py_binary")

DEFAULT_PYTHON_LOGGING_CONF = "@ros_comm//:tools/rosgraph/conf/python_logging.conf"

def ros_launch(name, nodes, launch_files):
    """Defines a ROS deployment.

    Args:
        name: The name of the deployment.
        nodes: The nodes used by the deployment.
        launch_files: The launch files used by the deployment.
    """
    launch_file_paths = ["'$(location {})'".format(x) for x in launch_files]
    substitutions = {
        "{default_python_logging_conf}": "$(location {})".format(
            DEFAULT_PYTHON_LOGGING_CONF,
        ),
        "{launch_files}": ", ".join(launch_file_paths),
    }

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "//ros:launch.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = [DEFAULT_PYTHON_LOGGING_CONF] + launch_files,
    )

    py_binary(
        name = name,
        srcs = [launch_script],
        data = nodes + launch_files,
        main = launch_script,
        visibility = ["//visibility:public"],
        deps = ["//third_party/legacy_roslaunch"],
    )
