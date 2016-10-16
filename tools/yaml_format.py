#!/usr/bin/python

import sys
import pprint
import ruamel.yaml
import argparse

parser = argparse.ArgumentParser(description='Prettify a yaml file')
parser.add_argument('-i', '--input', metavar='FILE', type=str, required=True , help='Set the input file')

#Setup the output argument (Automatically opens the input string as a file)
parser.add_argument('-o', '--output', metavar='FILE', type=argparse.FileType('w',0), required=True , help='Set the output file')

args = parser.parse_args()

#load the yaml file named by args.input
i = ruamel.yaml.round_trip_load(open(args.input, 'r'))

#Dump the Prettified yaml to file
o = ruamel.yaml.round_trip_dump(i, default_style='', indent=4, default_flow_style=False)

#Write the output to the file
args.output.write(o)

#Close the output file
args.output.close()
