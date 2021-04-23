workspace(name = "com_github_mvukov_rules_ros")

load("//repositories:repositories.bzl", "ros_repositories")

ros_repositories()

load("//repositories:deps.bzl", "ros_deps")

PYTHON_INTERPRETER = "python3.8"

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
    sha256 = "95d39fd84ff4474babaf190450ee034d958202043e366b9fc38f438c9e6c3334",
    strip_prefix = "rules_docker-0.16.0",
    urls = ["https://github.com/bazelbuild/rules_docker/releases/download/v0.16.0/rules_docker-v0.16.0.tar.gz"],
)

load(
    "@io_bazel_rules_docker//repositories:repositories.bzl",
    container_repositories = "repositories",
)

container_repositories()

load("@io_bazel_rules_docker//repositories:deps.bzl", container_deps = "deps")

container_deps()

load(
    "@io_bazel_rules_docker//container:container.bzl",
    "container_pull",
)

container_pull(
    name = "ros_deploy_base",
    digest = "sha256:54967c8f59e8607cd4a40c0d614b3391bf71112482f2e344d93ff455f60b3723",
    registry = "docker.io",
    repository = "mvukov/ros-deploy-base",
)

load(
    "//build_tools/toolchains:toolchains.bzl",
    "register_all_toolchains",
    "toolchain_repositories",
)

toolchain_repositories()

register_all_toolchains()
