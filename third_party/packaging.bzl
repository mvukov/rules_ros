load("@bazel_skylib//lib:dicts.bzl", "dicts")
load(
    "@bazel_tools//tools/build_defs/hash:hash.bzl",
    _hash_tools = "tools",
    _sha256 = "sha256",
)
load(
    "@io_bazel_rules_docker//skylib:zip.bzl",
    _gzip = "gzip",
    _zip_tools = "tools",
)

def _binary_name(ctx):
    # For //foo/bar/baz:blah this would translate to
    # /app/foo/bar/baz/blah
    return "/".join([
        ctx.attr.directory,
        ctx.attr.binary.label.package,
        ctx.attr.binary.label.name,
    ])

def _runfiles_dir(ctx):
    # For @foo//bar/baz:blah this would translate to
    # /app/bar/baz/blah.runfiles
    return _binary_name(ctx) + ".runfiles"

# The directory relative to which all ".short_path" paths are relative.

def _reference_dir(ctx):
    # For @foo//bar/baz:blah this would translate to
    # /app/bar/baz/blah.runfiles/foo
    return "/".join([_runfiles_dir(ctx), ctx.workspace_name])

# The special "external" directory which is an alternate way of accessing
# other repositories.

def _external_dir(ctx):
    # For @foo//bar/baz:blah this would translate to
    # /app/bar/baz/blah.runfiles/foo/external
    return "/".join([_reference_dir(ctx), "external"])

# The final location that this file needs to exist for the foo_binary target to
# properly execute.

def _final_emptyfile_path(ctx, name):
    if not name.startswith("external/"):
        # Names that don't start with external are relative to our own workspace.
        return _reference_dir(ctx) + "/" + name

    # References to workspace-external dependencies, which are identifiable
    # because their path begins with external/, are inconsistent with the
    # form of their File counterparts, whose ".short_form" is relative to
    #    .../foo.runfiles/workspace-name/  (aka _reference_dir(ctx))
    # whereas we see:
    #    external/foreign-workspace/...
    # so we "fix" the empty files' paths by removing "external/" and basing them
    # directly on the runfiles path.

    return "/".join([_runfiles_dir(ctx), name[len("external/"):]])

# The final location that this file needs to exist for the foo_binary target to
# properly execute.

def _final_file_path(ctx, f):
    return "/".join([_reference_dir(ctx), f.short_path])

# The foo_binary independent location in which we store a particular dependency's
# file such that it can be shared.

def _layer_emptyfile_path(ctx, name):
    if not name.startswith("external/"):
        # Names that don't start with external are relative to our own workspace.
        return "/".join([ctx.attr.directory, ctx.workspace_name, name])

    # References to workspace-external dependencies, which are identifiable
    # because their path begins with external/, are inconsistent with the
    # form of their File counterparts, whose ".short_form" is relative to
    #    .../foo.runfiles/workspace-name/  (aka _reference_dir(ctx))
    # whereas we see:
    #    external/foreign-workspace/...
    # so we "fix" the empty files' paths by removing "external/" and basing them
    # directly on the runfiles path.

    return "/".join([ctx.attr.directory, name[len("external/"):]])

# The foo_binary independent location in which we store a particular dependency's
# file such that it can be shared.

def layer_file_path(ctx, f):
    return "/".join([ctx.attr.directory, ctx.workspace_name, f.short_path])

def _default_runfiles(dep):
    return dep[DefaultInfo].default_runfiles.files

def _default_emptyfiles(dep):
    return dep[DefaultInfo].default_runfiles.empty_filenames

def _default_symlinks(dep):
    return dep[DefaultInfo].default_runfiles.symlinks

def _build_package(
        ctx,
        package,
        file_map = None,
        empty_files = None,
        empty_dirs = None,
        directory = None,
        symlinks = None):
    """Build the package.

    Args:
       ctx: The context
       name: The name of the package
       package: The output location for this package.
       file_map: Map of files to include in package (source to dest inside package)
       empty_files: List of empty files in the package
       empty_dirs: List of empty dirs in the package
       directory: Directory in which to store the file inside the package
       symlinks: List of symlinks to include in the package

    Returns:
       The package tar and its sha256 digest
    """
    toolchain_info = ctx.toolchains["@io_bazel_rules_docker//toolchains/docker:toolchain_type"].info
    name = ctx.label.name
    build_tar_exec = ctx.executable._build_tar
    mode = "0o555"  # Set the mode of files.
    args = ctx.actions.args()
    args.add(package, format = "--output=%s")
    args.add(directory, format = "--directory=%s")
    args.add(mode, format = "--mode=%s")

    if ctx.attr.mtime != _DEFAULT_MTIME:  # Note: Must match default in rule def.
        if ctx.attr.portable_mtime:
            fail("You may not set both mtime and portable_mtime")
        args.add(ctx.attr.mtime, format = "--mtime=%s")
    if ctx.attr.portable_mtime:
        args.add("--mtime=portable")
    if ctx.attr.enable_mtime_preservation:
        args.add("--enable_mtime_preservation=true")
    if toolchain_info.xz_path != "":
        args.add(toolchain_info.xz_path, format = "--xz_path=%s")

    all_files = [struct(src = f.path, dst = path) for (path, f) in file_map.items()]
    manifest = struct(
        files = all_files,
        symlinks = [struct(linkname = k, target = symlinks[k]) for k in symlinks],
        empty_files = empty_files or [],
        empty_dirs = empty_dirs or [],
        empty_root_dirs = [],
        tars = [],
        debs = [],
    )
    manifest_file = ctx.actions.declare_file(name + ".manifest")
    ctx.actions.write(manifest_file, manifest.to_json())
    args.add(manifest_file, format = "--manifest=%s")

    ctx.actions.run(
        executable = build_tar_exec,
        arguments = [args],
        tools = file_map.values() + [manifest_file],
        outputs = [package],
        use_default_shell_env = True,
        mnemonic = "BinaryPkgTar",
    )
    return package, _sha256(ctx, package)

def _zip_package(ctx, package, compression = "", compression_options = None):
    """Generate the zipped package.

    Args:
       ctx: The context
       package: File, package tar
       compression: str, compression mode, eg "gzip"
       compression_options: str, command-line options for the compression tool

    Returns:
       The zipped package tar and its sha256 digest
    """
    compression_options = compression_options or []
    if compression == "gzip":
        zipped_package = _gzip(ctx, package, options = compression_options)
    else:
        fail(
            'Unrecognized compression method (need "gzip"): %r' % compression,
            attr = "compression",
        )

    return zipped_package, _sha256(ctx, zipped_package)

def _binary_pkg_tar_impl(ctx):
    runfiles = _default_runfiles
    emptyfiles = _default_emptyfiles

    filepath = _final_file_path
    emptyfilepath = _final_emptyfile_path
    dep = ctx.attr.binary

    # Compute the set of remaining runfiles to include into the package.
    # runfiles(dep) can be `depset` or `list`. Convert it to list only if needed.
    runfiles_list = runfiles(dep).to_list() if type(runfiles(dep)) == "depset" else runfiles(dep)
    file_map = {
        filepath(ctx, f): f
        for f in runfiles_list
    }

    # emptyfiles(dep) can be `depset` or `list`. Convert it to list only if needed.
    emptyfiles_list = emptyfiles(dep).to_list() if type(emptyfiles(dep)) == "depset" else emptyfiles(dep)
    empty_files = [
        emptyfilepath(ctx, f)
        for f in emptyfiles_list
    ]

    symlinks = {}

    # Create symlinks to the runfiles path.
    # Include any symlinks from the runfiles of the target for which we are synthesizing the layer.
    symlinks.update({
        (_reference_dir(ctx) + "/" + s.path): layer_file_path(ctx, s.target_file)
        for s in _default_symlinks(dep).to_list()
        if hasattr(s, "path")  # "path" and "target_file" are exposed to starlark since bazel 0.21.0.
    })
    symlinks.update({
        _final_file_path(ctx, f): layer_file_path(ctx, f)
        for f in runfiles_list
        if _final_file_path(ctx, f) not in file_map
    })
    symlinks.update({
        _final_emptyfile_path(ctx, f): _layer_emptyfile_path(ctx, f)
        for f in emptyfiles_list
        if _final_emptyfile_path(ctx, f) not in empty_files
    })
    symlinks.update({
        # Create a directory symlink from <workspace>/external to the runfiles
        # root, since they may be accessed via either path.
        _external_dir(ctx): _runfiles_dir(ctx),
    })

    launcher_name = ctx.label.name + "_launcher.sh"
    launcher = ctx.actions.declare_file(launcher_name)
    workdir = "/".join([_runfiles_dir(ctx), ctx.workspace_name])
    ctx.actions.expand_template(
        template = ctx.file._launcher,
        output = launcher,
        substitutions = {
            "${workdir}": workdir,
            "${binary}": _final_file_path(ctx, ctx.executable.binary)
        },
        is_executable = True,
    )
    file_map.update({_binary_name(ctx): launcher})

    package = ctx.actions.declare_file(ctx.label.name + ".tar")
    package, package_sha256_sum = _build_package(
        ctx = ctx,
        package = package,
        file_map = file_map,
        empty_files = empty_files,
        empty_dirs = [],
        directory = "/",
        symlinks = symlinks,
    )

    zipped_package, zipped_package_sha256_sum = _zip_package(
        ctx,
        package,
        compression = ctx.attr.compression,
        compression_options = ctx.attr.compression_options,
    )

    return [DefaultInfo(files=depset([package, zipped_package]))]

_DEFAULT_MTIME = -1

binary_pkg_tar = rule(
    attrs = dicts.add({
        "binary": attr.label(
            executable = True,
            cfg = "target",
            mandatory = True,
            doc = "The binary target for which we are assembling the package.",
        ),
        "compression": attr.string(default = "gzip"),
        "compression_options": attr.string_list(),
        "directory": attr.string(
            default = "/app",
            doc = "Target directory.",
        ),
        "enable_mtime_preservation": attr.bool(default = False),
        "mtime": attr.int(default = _DEFAULT_MTIME),
        "portable_mtime": attr.bool(default = False),
        "_build_tar": attr.label(
            default = Label("@io_bazel_rules_docker//container:build_tar"),
            cfg = "host",
            executable = True,
            allow_files = True,
        ),
        "_launcher": attr.label(
            default = Label("@com_github_mvukov_rules_ros//third_party:launcher.sh.tpl"),
            allow_single_file = True,
        ),
    }, _hash_tools, _zip_tools),
    implementation = _binary_pkg_tar_impl,
    toolchains = ["@io_bazel_rules_docker//toolchains/docker:toolchain_type"],
)
