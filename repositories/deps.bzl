"""Configures repo dependencies.
"""

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")
load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

def ros_deps():
    """ Sets up ROS deps.
    """
    bazel_skylib_workspace()

    boost_deps()

    rules_foreign_cc_dependencies()
