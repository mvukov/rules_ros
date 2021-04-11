load(
    "//build_tools/toolchains/gcc-linaro-7.5.0-aarch64-linux:repositories.bzl",
    gcc_linaro_7_5_0_aarch64_linux_repositories = "toolchain_repositories",
)

def toolchain_repositories():
    gcc_linaro_7_5_0_aarch64_linux_repositories()

def register_all_toolchains():
    native.register_toolchains(
        "//build_tools/toolchains/gcc-linaro-7.5.0-aarch64-linux:toolchain_aarch64_linux",
        # "//build_tools/toolchains/gcc-linaro-7.5.0-aarch64-linux:py3_toolchain",
    )
