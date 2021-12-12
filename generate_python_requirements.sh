#!/bin/bash

set -e

PYTHON_INTERPRETER="python3.8"

cd "$(bazel info workspace)"
rm requirements_lock.txt

_ARGS=(
  --allow-unsafe
  --generate-hashes
  --output-file=requirements_lock.txt
  requirements.txt
)
${PYTHON_INTERPRETER} -m piptools compile ${_ARGS[@]}
