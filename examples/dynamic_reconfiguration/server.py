import sys, pprint

pprint.pprint(sys.path)

import dynamic_reconfigure.msg

print(">>", dynamic_reconfigure.msg.__file__)
from tutorial_cfg.cfg import TutorialConfig
