#!/usr/bin/python

import sys
import ruamel.yaml
import argparse

# This function overrides the default behavior for writing back strings
#   The modified behavior is to add " around strings
def process_scalar(self):
    if self.analysis is None:
        self.analysis = self.analyze_scalar(self.event.value)
    if self.style is None:
        self.style = self.choose_scalar_style()
    split = (not self.simple_key_context)
    # Start of added section
    if split:  # not a key
        is_string = True
        if self.event.value and self.event.value.lstrip('-').replace('.','',1).isdigit():
            is_string = False
        if is_string:
            self.style = '"'
    # End of added section
    if self.analysis.multiline and split    \
            and (not self.style or self.style in '\'\"'):
        self.write_indent()
    if self.style == '"':
        self.write_double_quoted(self.analysis.scalar, split)
    elif self.style == '\'':
        self.write_single_quoted(self.analysis.scalar, split)
    elif self.style == '>':
        self.write_folded(self.analysis.scalar)
    elif self.style == '|':
        self.write_literal(self.analysis.scalar)
    else:
        self.write_plain(self.analysis.scalar, split)
    self.analysis = None
    self.style = None
    if self.event.comment:
        self.write_post_comment(self.event)

#This function is a recursive way to traverse through the nested dictionaries
#  of yaml files.
# x is the dictionary to take values from
# y is the dictionary to update the values of
def recurse(x,y):
    for k,v in x.items():
        if isinstance(v,dict):
            if k in y:
                recurse(v,y[k])
        else:
            # If the key is in the destination overwrite else do nothing
            if k in y:
                y[k] = v

if __name__ == '__main__':
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
    recurse(i, initial)

    #Override the Dumper's behavior with strings
    dd = ruamel.yaml.RoundTripDumper
    dd.process_scalar = process_scalar

    #Dump the Prettified yaml to file
    o = ruamel.yaml.dump(initial, default_style='', indent=4, Dumper=dd,
                                    block_seq_indent=1, default_flow_style=False)

    outputFile = open(args.output, 'w')

    #Write the output to the file
    outputFile.write(o)

    #Close the output file
    outputFile.close()
