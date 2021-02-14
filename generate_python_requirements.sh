#!/bin/bash

set -e

PYTHON_INTERPRETER="python3.8"

cd "$(bazel info workspace)"
${PYTHON_INTERPRETER} -m poetry update
${PYTHON_INTERPRETER} -m poetry export --format requirements.txt --output requirements.txt
