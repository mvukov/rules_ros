""" Implements a macro for setting up pylint-based linter targets.
"""

load("@rules_ros_pip_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_test")

def _is_py_source_label(label):
    return label.endswith(".py")

def _add_linter_rule(source_labels, source_filenames, name):
    pylint_cfg = "//:pylint.rc"

    py_test(
        name = name + "_pylint",
        srcs = ["//build_tools:run_pylint.py"],
        data = [pylint_cfg] + source_labels,
        args = source_filenames,
        main = "//build_tools:run_pylint.py",
        size = "small",
        tags = ["pylint", "lint"],
        deps = [
            requirement("pylint"),
            # Make the target as dependency such that the linter test target the
            # gets correct PYTHONPATH.
            ":{}".format(name),
        ],
    )

def pylint():
    """Adds test targets for linting Python files.

    For every rule in the build file in the current directory, adds a test rule
    that runs pylint over the Python source files listed in that rule.
    """
    existing_rules = native.existing_rules().values()

    for rule in existing_rules:
        if "nolint" in rule.get("tags"):
            # Disable linting when requested (e.g. for generated code).
            continue

        source_labels = [
            label
            for label in rule.get("srcs", [])
            if _is_py_source_label(label)
        ]
        source_filenames = ["$(location {})".format(x) for x in source_labels]

        # Run the pylint checker as a unit test.
        if len(source_filenames) > 0:
            _add_linter_rule(
                source_labels,
                source_filenames,
                rule["name"],
            )
