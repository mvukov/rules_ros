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
        binary = binary,
        base = base,
        **kwargs
    )
