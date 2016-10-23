#!/usr/bin/python

import sys
import pprint
import ruamel.yaml
import argparse

parser = argparse.ArgumentParser(description='Prettify a yaml file')
parser.add_argument('-i', '--input', metavar='FILE', type=str, required=True,
                    help='Set the input file')

#Setup the output argument (Automatically opens the input string as a file)
parser.add_argument('-o', '--output', metavar='FILE', type=str, required=True,
                    help='Set the output file')

args = parser.parse_args()

# the input and output files must be different
# Exit if they are the same
if args.input == args.output:
    print ("[\033[91mFATAL\033[0m] input and output files are the same!\nExiting...")
    sys.exit()

outputFile = open(args.output, 'r')

#Read the output file
initial = ruamel.yaml.round_trip_load(outputFile)
outputFile.close()

#load the yaml file named by args.input
i = ruamel.yaml.round_trip_load(open(args.input, 'r'))

# Do parameter replacement here
initial['control']['proportional']['psi'] = i['control']['proportional']['psi']

#Dump the Prettified yaml to file
o = ruamel.yaml.round_trip_dump(initial, default_style='', indent=4, default_flow_style=False)

outputFile = open(args.output, 'w')

#Write the output to the file
outputFile.write(o)

#Close the output file
outputFile.close()
