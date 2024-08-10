import multiprocessing
import os
import re
import sys

# Prevent pylint using its persistent data.
# Must be set before pylint is imported!
os.environ['PYLINTHOME'] = os.environ['TEST_UNDECLARED_OUTPUTS_DIR']

import pylint.lint
import pylint.reporters.text

PYLINTRC_FILE = './pylint.rc'
NUM_CPU_CORES_FOR_PYLINT = 2


class FileLinter:
    """Implements a file linter."""

    def __init__(self, python_path):
        self.pylint_file = PYLINTRC_FILE
        self.init_hook_arg = f'import sys; sys.path = {python_path!s}'

    def __call__(self, file_to_lint):

        class TextReporterBuffer:
            """Stores the output produced by the pylint TextReporter."""

            def __init__(self):
                self.content = []

            def write(self, input_str):
                self.content.append(input_str)

            def read(self):
                return self.content

        pylint_output = TextReporterBuffer()
        pylint_args = [
            '--rcfile=' + self.pylint_file, '-rn', '-sn', file_to_lint,
            '--init-hook', self.init_hook_arg
        ]
        pylint.lint.Run(
            pylint_args,
            reporter=pylint.reporters.text.TextReporter(pylint_output),
            exit=False)

        pylint_errors = []
        for output_line in pylint_output.read():
            # Look for Error, Convention or Warning messages in pylint output.
            if re.search(r'(E|C|W)\d{4}:', output_line):
                pylint_errors.append(output_line)

        return pylint_errors


def main():
    files_to_lint = sys.argv[1:]
    if not files_to_lint:
        return 0
    pool = multiprocessing.Pool(NUM_CPU_CORES_FOR_PYLINT)
    file_linter = FileLinter(python_path=sys.path)

    linting_output = pool.map(file_linter, files_to_lint)

    success = True
    for linter_output in linting_output:
        if not linter_output:
            continue

        print('-' * 80)
        for line in linter_output:
            print(line)
        success = False

    if success:
        sys.exit(0)
    sys.exit(1)


if __name__ == '__main__':
    main()
