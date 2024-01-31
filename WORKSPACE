workspace(name = "com_github_mvukov_rules_ros")

load("//repositories:repositories.bzl", "ros_repositories")

ros_repositories()

load("//repositories:deps.bzl", "ros_deps")

PYTHON_INTERPRETER = "python3.7"

ros_deps(
    python_interpreter = PYTHON_INTERPRETER,
    python_requirements_lock = "//:requirements_lock.txt",
)

load(
    "@rules_ros_pip_deps//:requirements.bzl",
    install_rules_ros_pip_deps = "install_deps",
)

install_rules_ros_pip_deps()

################################################################################
#
# Below are handled deps for internal development and/or examples.
#
################################################################################

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")


http_archive(
    name = "io_bazel_rules_docker",
    sha256 = "b1e80761a8a8243d03ebca8845e9cc1ba6c82ce7c5179ce2b295cd36f7e394bf",
    urls = ["https://github.com/bazelbuild/rules_docker/releases/download/v0.25.0/rules_docker-v0.25.0.tar.gz"],
)

load(
    "@io_bazel_rules_docker//toolchains/docker:toolchain.bzl",
    docker_toolchain_configure = "toolchain_configure",
)

docker_toolchain_configure(
    name = "docker_config",
    docker_path = "/usr/bin/docker",
    xz_path = "usr/bin/xz",
)

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

load(
    "@io_bazel_rules_docker//python3:image.bzl",
    _py_image_repos = "repositories",
)

_py_image_repos()

load(
    "@io_bazel_rules_docker//go:image.bzl",
    _go_image_repos = "repositories",
)

_go_image_repos()

load("@io_bazel_rules_docker//container:container.bzl", "container_pull")

container_pull(
    name = "ros_deploy_base",
    digest = "sha256:54967c8f59e8607cd4a40c0d614b3391bf71112482f2e344d93ff455f60b3723",
    registry = "docker.io",
    repository = "mvukov/ros-deploy-base",
)

http_archive(
    name = "io_bazel_stardoc",
    sha256 = "aa814dae0ac400bbab2e8881f9915c6f47c49664bf087c409a15f90438d2c23e",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/stardoc/releases/download/0.5.1/stardoc-0.5.1.tar.gz",
        "https://github.com/bazelbuild/stardoc/releases/download/0.5.1/stardoc-0.5.1.tar.gz",
    ],
)

http_archive(
    name = "aspect_bazel_lib",
    sha256 = "e00a57d37a8d8b629951e43d1af9b079429b6ea9710752f08910f13afdb825f0",
    strip_prefix = "bazel-lib-1.10.1",
    url = "https://github.com/aspect-build/bazel-lib/archive/refs/tags/v1.10.1.tar.gz",
)

load("@aspect_bazel_lib//lib:repositories.bzl", "aspect_bazel_lib_dependencies")

aspect_bazel_lib_dependencies()

http_archive(
    name = "bazel_gcc_toolchain",
    sha256 = "4587cac066f970877fda621a8b1480e333bd05cf5165cae5389acaa4327f6471",
    strip_prefix = "gcc-toolchain-76f3cb579505c93363efec5c69bd00f1ae4285d0",
    urls = ["https://github.com/aspect-build/gcc-toolchain/archive/76f3cb579505c93363efec5c69bd00f1ae4285d0.zip"],
)

load("@bazel_gcc_toolchain//toolchain:repositories.bzl", gcc_toolchain_repositories = "gcc_toolchain_dependencies")

gcc_toolchain_repositories()

load("@bazel_gcc_toolchain//toolchain:defs.bzl", "gcc_register_toolchain")

# This is GCC 7.3, used to ensure compatibility with Ubuntu 18.04 / Jetpack 4.6 for Jetson Nano.
gcc_register_toolchain(
    name = "gcc_toolchain_aarch64",
    sha256 = "a857e31461e133265e6dc0618b5e1c5f8d19002e5f8f49b50144a8c7fa787533",
    strip_prefix = "aarch64--glibc--bleeding-edge-2018.02-1",
    target_arch = "aarch64",
    url = "https://toolchains.bootlin.com/downloads/releases/toolchains/aarch64/tarballs/aarch64--glibc--bleeding-edge-2018.02-1.tar.bz2",
)
