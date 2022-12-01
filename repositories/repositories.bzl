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
        sha256 = "c03246c11efd49266e8e41e12931090b613e12a59e6f55ba2efd29a7cb8b4258",
        strip_prefix = "rules_python-0.11.0",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.11.0.tar.gz",
    )

    maybe(
        http_archive,
        name = "bazel_skylib",
        urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/1.0.3/bazel-skylib-1.0.3.tar.gz"],
        sha256 = "1c531376ac7e5a180e0237938a2536de0c54d93f5c278634818e0efc952dd56c",
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
        name = "com_google_googletest",
        sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",
        strip_prefix = "googletest-release-1.11.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/release-1.11.0.tar.gz"],
    )

    maybe(
        http_archive,
        name = "rules_foreign_cc",
        sha256 = "4f2207a83c500a16c9095a224acc4ec8c1406c6febd0a042b3ac15ef6ff0c640",
        strip_prefix = "rules_foreign_cc-83e6cf48cf00214bb4348691beecef8f3bc42f7d",
        urls = ["https://github.com/bazelbuild/rules_foreign_cc/archive/83e6cf48cf00214bb4348691beecef8f3bc42f7d.zip"],
    )

    maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        strip_prefix = "rules_boost-4546e073fa5d493806f51537ccc5174403f3bd63",
        sha256 = "0a9d91543f06f2510f760fe94e433d045f02d185ba763b6989d9122b52f45afb",
        urls = ["https://github.com/nelhage/rules_boost/archive/4546e073fa5d493806f51537ccc5174403f3bd63.zip"],
    )

    maybe(
        http_archive,
        name = "console_bridge",
        build_file = "@com_github_mvukov_rules_ros//repositories:console_bridge.BUILD.bazel",
        sha256 = "2ff175a9bb2b1849f12a6bf972ce7e4313d543a2bbc83b60fdae7db6e0ba353f",
        strip_prefix = "console_bridge-1.0.1",
        urls = ["https://github.com/ros/console_bridge/archive/1.0.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "roscpp_core",
        build_file = "@com_github_mvukov_rules_ros//repositories:roscpp_core.BUILD.bazel",
        sha256 = "a2aa77814ed97b48995c872a405c51f6b0f1ab9d40e38ece483852bbd273ad7b",
        strip_prefix = "roscpp_core-0.7.2",
        urls = ["https://github.com/ros/roscpp_core/archive/0.7.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "rosconsole",
        build_file = "@com_github_mvukov_rules_ros//repositories:rosconsole.BUILD.bazel",
        sha256 = "0b2cbc4f9a92466c0fbae7863482b286ef87692de4941527cb429e6c74639246",
        strip_prefix = "rosconsole-1.14.3",
        urls = ["https://github.com/ros/rosconsole/archive/1.14.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genmsg",
        build_file = "@com_github_mvukov_rules_ros//repositories:genmsg.BUILD.bazel",
        sha256 = "0e414846823a2aaa7781f81268251c7c9a45ff96cef8e6a78bbbbcf7e4c28d56",
        strip_prefix = "genmsg-0.5.16",
        urls = ["https://github.com/ros/genmsg/archive/0.5.16.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_gencpp",
        build_file = "@com_github_mvukov_rules_ros//repositories:gencpp.BUILD.bazel",
        sha256 = "05acfeeb1bbc374356bf7674fee2a7aab3bf6a48ebad4a06fd0f0d4455a60720",
        strip_prefix = "gencpp-0.6.5",
        urls = ["https://github.com/ros/gencpp/archive/0.6.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_genpy",
        build_file = "@com_github_mvukov_rules_ros//repositories:genpy.BUILD.bazel",
        sha256 = "523d20068171ce7e5b4c453eba7976aafa819e6b5af806ffdf6bc1d7a1dfc2a8",
        strip_prefix = "genpy-0.6.14",
        urls = ["https://github.com/ros/genpy/archive/0.6.14.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_std_msgs",
        build_file = "@com_github_mvukov_rules_ros//repositories:std_msgs.BUILD.bazel",
        sha256 = "ee6592d37b00a94cab8216aac2cfb5120f6da09ffa94bfe197fe8dc76dd21326",
        strip_prefix = "std_msgs-0.5.13",
        urls = ["https://github.com/ros/std_msgs/archive/0.5.13.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_comm_msgs",
        build_file = "@com_github_mvukov_rules_ros//repositories:ros_comm_msgs.BUILD.bazel",
        sha256 = "5b8b91e8671d03ea84ba32a3ea7360bc4594655e7ba3ec6677a984f393aaafbd",
        strip_prefix = "ros_comm_msgs-1.11.3",
        urls = ["https://github.com/ros/ros_comm_msgs/archive/1.11.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "lz4",
        build_file = "@com_github_mvukov_rules_ros//repositories:lz4.BUILD.bazel",
        sha256 = "030644df4611007ff7dc962d981f390361e6c97a34e5cbc393ddfbe019ffe2c1",
        strip_prefix = "lz4-1.9.3",
        urls = ["https://github.com/lz4/lz4/archive/v1.9.3.tar.gz"],
    )

    maybe(
        http_archive,
        name = "bzip2",
        build_file = "@com_github_mvukov_rules_ros//repositories:bzip2.BUILD.bazel",
        sha256 = "ab5a03176ee106d3f0fa90e381da478ddae405918153cca248e682cd0c4a2269",
        strip_prefix = "bzip2-1.0.8",
        urls = ["https://sourceware.org/pub/bzip2/bzip2-1.0.8.tar.gz"],
    )

    # Branched off ros_comm 1.15.9. Changes are in branch feature/rules_ros.
    maybe(
        http_archive,
        name = "ros_comm",
        build_file = "@com_github_mvukov_rules_ros//repositories:ros_comm.BUILD.bazel",
        sha256 = "65906eab18bdf5832af7b6de8c21e3d87d3fac9732a1219637f5b6f5c948508d",
        strip_prefix = "ros_comm-30922861f247937001f82094c0934a6b802b6325",
        urls = ["https://github.com/mvukov/ros_comm/archive/30922861f247937001f82094c0934a6b802b6325.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_ros",
        build_file = "@com_github_mvukov_rules_ros//repositories:ros.BUILD.bazel",
        sha256 = "7a1e729de9be807862b6ed721475fec74583f6dc0c06b233b06b1b9fda31291e",
        strip_prefix = "ros-1.15.7",
        urls = ["https://github.com/ros/ros/archive/1.15.7.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_common_msgs",
        build_file = "@com_github_mvukov_rules_ros//repositories:common_msgs.BUILD.bazel",
        sha256 = "74af8cc88bdc9c23cbc270d322e50562857e2c877359423f389d51c0735ee230",
        strip_prefix = "common_msgs-1.13.1",
        urls = ["https://github.com/ros/common_msgs/archive/1.13.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_actionlib",
        build_file = "@com_github_mvukov_rules_ros//repositories:actionlib.BUILD.bazel",
        sha256 = "b741755881e30b9aea6bcdd9831e3f0932a8bbba02fa59e5c0e5970280024055",
        strip_prefix = "actionlib-1.13.2",
        urls = ["https://github.com/ros/actionlib/archive/1.13.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_dynamic_reconfigure",
        build_file = "@com_github_mvukov_rules_ros//repositories:dynamic_reconfigure.BUILD.bazel",
        sha256 = "7cdb46269ae222a0ed5632d7c9b9d6f1e351c35dc9cc14b48930ad89273a5da5",
        strip_prefix = "dynamic_reconfigure-1.7.1",
        urls = ["https://github.com/ros/dynamic_reconfigure/archive/refs/tags/1.7.1.tar.gz"],
    )

    ############################################################################

    maybe(
        http_archive,
        name = "eigen",
        build_file = "@com_github_mvukov_rules_ros//repositories:eigen.BUILD.bazel",
        sha256 = "7985975b787340124786f092b3a07d594b2e9cd53bbfe5f3d9b1daee7d55f56f",
        strip_prefix = "eigen-3.3.9",
        urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz"],
    )

    maybe(
        http_archive,
        name = "ros_geometry2",
        build_file = "@com_github_mvukov_rules_ros//repositories:geometry2.BUILD.bazel",
        sha256 = "0b5d461c71d6dc1dbdb99a2ba39e1515194cd451c2e53d53caadb3ecea13367a",
        strip_prefix = "geometry2-0.7.5",
        urls = ["https://github.com/ros/geometry2/archive/0.7.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "orocos_kdl",
        build_file = "@com_github_mvukov_rules_ros//repositories:orocos_kdl.BUILD.bazel",
        sha256 = "5acb90acd82b10971717aca6c17874390762ecdaa3a8e4db04984ea1d4a2af9b",
        strip_prefix = "orocos_kinematics_dynamics-1.5.1",
        urls = ["https://github.com/orocos/orocos_kinematics_dynamics/archive/refs/tags/v1.5.1.tar.gz"],
    )

    maybe(
        http_archive,
        name = "urdfdom_headers",
        build_file = "@com_github_mvukov_rules_ros//repositories:urdfdom_headers.BUILD.bazel",
        sha256 = "76a68657c38e54bb45bddc4bd7d823a3b04edcd08064a56d8e7d46b9912035ac",
        strip_prefix = "urdfdom_headers-1.0.5",
        urls = ["https://github.com/ros/urdfdom_headers/archive/refs/tags/1.0.5.tar.gz"],
    )

    maybe(
        http_archive,
        name = "tinyxml",
        build_file = "@com_github_mvukov_rules_ros//repositories:tinyxml.BUILD.bazel",
        sha256 = "15bdfdcec58a7da30adc87ac2b078e4417dbe5392f3afb719f9ba6d062645593",
        urls = [
            "https://sourceforge.net/projects/tinyxml/files/tinyxml/2.6.2/tinyxml_2_6_2.tar.gz",
            "http://archive.ubuntu.com/ubuntu/pool/universe/t/tinyxml/tinyxml_2.6.2.orig.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "urdfdom",
        build_file = "@com_github_mvukov_rules_ros//repositories:urdfdom.BUILD.bazel",
        sha256 = "8f3d56b0cbc4b84436d8baf4c8346cd2ee7ffb257bba5ddd9892c41bf516edc4",
        strip_prefix = "urdfdom-1.0.4",
        urls = ["https://github.com/ros/urdfdom/archive/refs/tags/1.0.4.tar.gz"],
    )
