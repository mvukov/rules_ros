"""Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")
load("@rules_python//python:pip.bzl", "pip_parse")

def ros_deps(python_interpreter, python_requirements_lock):
    """ Sets up ROS deps.

    Args:
      python_interpreter: The Python interpreter.
      python_requirements_lock: The transitively locked set of Python requirements.
    """
    if not native.existing_rule("rules_ros_pip_deps"):
        pip_parse(
            name = "rules_ros_pip_deps",
            python_interpreter = python_interpreter,
            requirements_lock = python_requirements_lock,
        )

    bazel_skylib_workspace()

    boost_deps()

    rules_foreign_cc_dependencies()
