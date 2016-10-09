#!/usr/bin/python

import sys
import yaml
import pprint

filename = sys.argv[1]

y = yaml.safe_load(open(filename, 'r'))

print yaml.dump(y, default_style='', indent=4, default_flow_style=False)
