load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

ALL_FILES_TEMPLATE = """
package(default_visibility = ["//visibility:public"])

filegroup(
  name = "{name}",
  srcs = glob([
    '**',
  ]),
)
"""

def toolchain_repositories():
    maybe(
        http_archive,
        name = "gcc-linaro-7.5.0-aarch64-linux",
        build_file_content = ALL_FILES_TEMPLATE.format(name = "toolchain"),
        sha256 = "3b6465fb91564b54bbdf9578b4cc3aa198dd363f7a43820eab06ea2932c8e0bf",
        strip_prefix = "gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu",
        url = "https://releases.linaro.org/components/toolchain/binaries/latest-7/aarch64-linux-gnu/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu.tar.xz",
    )

    maybe(
        http_archive,
        name = "sysroot-glibc-linaro-2.25-aarch64-linux",
        build_file_content = ALL_FILES_TEMPLATE.format(name = "sysroot"),
        sha256 = "6ac110843a7f96d12451a09907d17f18c7b279b5beb5493988da7532fd91ae86",
        strip_prefix = "sysroot-glibc-linaro-2.25-2019.12-aarch64-linux-gnu",
        url = "https://releases.linaro.org/components/toolchain/binaries/latest-7/aarch64-linux-gnu/sysroot-glibc-linaro-2.25-2019.12-aarch64-linux-gnu.tar.xz",
    )
