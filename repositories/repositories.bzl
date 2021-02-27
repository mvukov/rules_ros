"""Handles import of external/third-party repositories.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def ros_repositories():
    """Imports external/third-party repositories.
    """

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "b6d46438523a3ec0f3cead544190ee13223a52f6a6765a29eae7b7cc24cc83a0",
        url = "https://github.com/bazelbuild/rules_python/releases/download/0.1.0/rules_python-0.1.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "com_github_gflags_gflags",
        sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
        strip_prefix = "gflags-2.2.2",
        urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_glog",
        sha256 = "62efeb57ff70db9ea2129a16d0f908941e355d09d6d83c9f7b18557c0a7ab59e",
        strip_prefix = "glog-d516278b1cd33cd148e8989aec488b6049a4ca0b",
        urls = ["https://github.com/google/glog/archive/d516278b1cd33cd148e8989aec488b6049a4ca0b.zip"],
    )

    maybe(
        http_archive,
        name = "console_bridge",
        build_file = "//repositories:console_bridge.BUILD.bazel",
        sha256 = "2ff175a9bb2b1849f12a6bf972ce7e4313d543a2bbc83b60fdae7db6e0ba353f",
        strip_prefix = "console_bridge-1.0.1",
        urls = ["https://github.com/ros/console_bridge/archive/1.0.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "roscpp_core",
        build_file = "//repositories:roscpp_core.BUILD.bazel",
        sha256 = "a2aa77814ed97b48995c872a405c51f6b0f1ab9d40e38ece483852bbd273ad7b",
        strip_prefix = "roscpp_core-0.7.2",
        urls = ["https://github.com/ros/roscpp_core/archive/0.7.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "rosconsole",
        build_file = "//repositories:rosconsole.BUILD.bazel",
        sha256 = "0b2cbc4f9a92466c0fbae7863482b286ef87692de4941527cb429e6c74639246",
        strip_prefix = "rosconsole-1.14.3",
        urls = ["https://github.com/ros/rosconsole/archive/1.14.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genmsg",
        build_file = "//repositories:genmsg.BUILD.bazel",
        sha256 = "0e414846823a2aaa7781f81268251c7c9a45ff96cef8e6a78bbbbcf7e4c28d56",
        strip_prefix = "genmsg-0.5.16",
        urls = ["https://github.com/ros/genmsg/archive/0.5.16.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_gencpp",
        build_file = "//repositories:gencpp.BUILD.bazel",
        sha256 = "05acfeeb1bbc374356bf7674fee2a7aab3bf6a48ebad4a06fd0f0d4455a60720",
        strip_prefix = "gencpp-0.6.5",
        urls = ["https://github.com/ros/gencpp/archive/0.6.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genpy",
        build_file = "//repositories:genpy.BUILD.bazel",
        sha256 = "523d20068171ce7e5b4c453eba7976aafa819e6b5af806ffdf6bc1d7a1dfc2a8",
        strip_prefix = "genpy-0.6.14",
        urls = ["https://github.com/ros/genpy/archive/0.6.14.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_std_msgs",
        build_file = "//repositories:std_msgs.BUILD.bazel",
        sha256 = "ee6592d37b00a94cab8216aac2cfb5120f6da09ffa94bfe197fe8dc76dd21326",
        strip_prefix = "std_msgs-0.5.13",
        urls = ["https://github.com/ros/std_msgs/archive/0.5.13.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_comm_msgs",
        build_file = "//repositories:ros_comm_msgs.BUILD.bazel",
        sha256 = "5b8b91e8671d03ea84ba32a3ea7360bc4594655e7ba3ec6677a984f393aaafbd",
        strip_prefix = "ros_comm_msgs-1.11.3",
        urls = ["https://github.com/ros/ros_comm_msgs/archive/1.11.3.tar.gz"],
    )

    # Branched off ros_comm 1.15.9. Changes are in branch feature/rules_ros.
    maybe(
        http_archive,
        name = "ros_comm",
        build_file = "//repositories:ros_comm.BUILD.bazel",
        sha256 = "91ac044dd0abfdd1ae7a01795446c5f674c0bbee8a535796fb0d549ab367456e",
        strip_prefix = "ros_comm-eeb8d32723092749b4a1be2bb1677ad5bbcb383b",
        urls = ["https://github.com/mvukov/ros_comm/archive/eeb8d32723092749b4a1be2bb1677ad5bbcb383b.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_ros",
        build_file = "//repositories:ros.BUILD.bazel",
        sha256 = "7a1e729de9be807862b6ed721475fec74583f6dc0c06b233b06b1b9fda31291e",
        strip_prefix = "ros-1.15.7",
        urls = ["https://github.com/ros/ros/archive/1.15.7.tar.gz"],
    )
