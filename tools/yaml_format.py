#!/usr/bin/python

import sys
import pprint
import ruamel.yaml

filename = sys.argv[1]

y = ruamel.yaml.round_trip_load(open(filename, 'r'))

print ruamel.yaml.round_trip_dump(y, default_style='', indent=4, default_flow_style=False)
