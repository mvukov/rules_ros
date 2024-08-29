""" Implements functionality for launching ROS deployments using roslaunch.
"""

load("@rules_python//python:defs.bzl", "py_binary")
load("//third_party:expand_template.bzl", "expand_template")

def ros_launch(name, nodes, launch_files, **kwargs):
    """ Defines a ROS deployment.

    Args:
        name: A unique target name.
        nodes: A list of ROS nodes for the deployment.
        launch_files: A list of roslaunch-compatible launch files.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-binaries
    """
    launch_file_paths = ["'$(rootpath {})'".format(x) for x in launch_files]
    substitutions = {
        "{launch_files}": ", ".join(launch_file_paths),
    }

    data = kwargs.pop("data", [])

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "@rules_ros//ros:launch.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = launch_files,
    )

    py_binary(
        name = name,
        srcs = [launch_script],
        data = nodes + launch_files + data,
        main = launch_script,
        deps = ["@rules_ros//third_party/ros:roslaunch"],
        **kwargs
    )
