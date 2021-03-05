load("@io_bazel_rules_docker//lang:image.bzl", "app_layer")

def container_image(name, binary, base, **kwargs):
    """Constructs a container image for the given binary.

    Args:
        name: Name of the target.
        binary: The binary to embed in the image.
        base: Base image to use for the image.
        **kwargs: TBD.
    """

    app_layer(
        name = name,
        base = base,
        entrypoint = ["/usr/bin/python3"],
        binary = binary,
        visibility = kwargs.get("visibility", None),
        tags = kwargs.get("tags", None),
        args = kwargs.get("args"),
        data = kwargs.get("data"),
        testonly = kwargs.get("testonly"),
        # The targets of the symlinks in the symlink layers are relative to the
        # workspace directory under the app directory. Thus, create an empty
        # workspace directory to ensure the symlinks are valid. See
        # https://github.com/bazelbuild/rules_docker/issues/161 for details.
        create_empty_workspace_dir = True,
        docker_run_flags = kwargs.get("docker_run_flags"),
    )
