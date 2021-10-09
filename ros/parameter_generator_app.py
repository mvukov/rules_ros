import argparse
import importlib.machinery
import importlib.util
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=str, required=True)
parser.add_argument('--ros_package_name', type=str, required=True)
args, generator_args = parser.parse_known_args()
sys.argv = sys.argv[:1] + generator_args

cfg_basename = os.path.basename(args.input)
module_name = os.path.splitext(cfg_basename)[0]
loader = importlib.machinery.SourceFileLoader(module_name, args.input)
spec = importlib.util.spec_from_loader(module_name, loader)
module = importlib.util.module_from_spec(spec)
loader.exec_module(module)

try:
    module.gen.generate(args.ros_package_name, module_name, module_name)
except AttributeError:
    sys.exit(
        'ERROR: {} must have "gen" generator variable!'.format(cfg_basename))
