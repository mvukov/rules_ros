""" Implements functionality for defining ROS tests using rostest.
"""

load("@rules_python//python:defs.bzl", "py_test")
load("//third_party:expand_template.bzl", "expand_template")

def ros_test(name, nodes, launch_file, node_path_override=[], **kwargs):
    """ Defines a ROS test.

    Args:Fd
        name: A unique target name.
        nodes: A list of ROS nodes used by the test.
        launch_file: A rostest-compatible launch file.
        **kwargs: https://bazel.build/reference/be/common-definitions#common-attributes-tests
    """
    launch_file_path = "'$(location {})'".format(launch_file)

    joined = []
    for override in node_path_override:
        joined.append("\"{}\"".format(override))
    substitutions = {
        "{launch_file}": launch_file_path,
        "{node_path_override}": ",".join(joined),
    }

    launch_script = "{}_launch.py".format(name)
    expand_template(
        name = "{}_launch_gen".format(name),
        template = "@com_github_mvukov_rules_ros//ros:test.py.tpl",
        substitutions = substitutions,
        out = launch_script,
        data = [launch_file],
    )

    py_test(
        name = name,
        srcs = [launch_script],
        data = nodes + [launch_file],
        main = launch_script,
        deps = ["@com_github_mvukov_rules_ros//third_party/legacy_rostest"],
        **kwargs
    )
