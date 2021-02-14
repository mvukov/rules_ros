"""Configures repo dependencies.
"""

load("@rules_python//python:pip.bzl", "pip_install")

def ros_deps(python_interpreter, python_requirements):
    if not native.existing_rule("rules_ros_pip_deps"):
        pip_install(
            name = "rules_ros_pip_deps",
            python_interpreter = python_interpreter,
            requirements = python_requirements,
        )
