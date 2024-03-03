workspace(name = "com_github_mvukov_rules_ros")

load("//repositories:repositories.bzl", "ros_repositories")

ros_repositories()

load("@rules_python//python:repositories.bzl", "py_repositories", "python_register_toolchains")

py_repositories()

python_register_toolchains(
    name = "rules_ros_python",
    python_version = "3.10",
)

load("@rules_python//python:pip.bzl", "pip_parse")
load("@rules_ros_python//:defs.bzl", python_interpreter_target = "interpreter")

pip_parse(
    name = "rules_ros_pip_deps",
    python_interpreter_target = python_interpreter_target,
    requirements_lock = "@com_github_mvukov_rules_ros//:requirements_lock.txt",
)

load(
    "@rules_ros_pip_deps//:requirements.bzl",
    install_rules_ros_pip_deps = "install_deps",
)

install_rules_ros_pip_deps()

load("//repositories:deps.bzl", "ros_deps")

ros_deps()


# Internal dependencies.

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "io_bazel_stardoc",
    sha256 = "3fd8fec4ddec3c670bd810904e2e33170bedfe12f90adf943508184be458c8bb",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/stardoc/releases/download/0.5.3/stardoc-0.5.3.tar.gz",
        "https://github.com/bazelbuild/stardoc/releases/download/0.5.3/stardoc-0.5.3.tar.gz",
    ],
)

http_archive(
    name = "aspect_bazel_lib",
    sha256 = "a59096e01b43d86c6667a869f0e90e0c4b1d4cb03c3d3a972a32ff687c750ac2",
    strip_prefix = "bazel-lib-2.5.1",
    url = "https://github.com/aspect-build/bazel-lib/releases/download/v2.5.1/bazel-lib-v2.5.1.tar.gz",
)

load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies", "aspect_bazel_lib_register_toolchains")

# Required bazel-lib dependencies

aspect_bazel_lib_dependencies()

# Register bazel-lib toolchains

aspect_bazel_lib_register_toolchains()
