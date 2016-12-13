#!/usr/bin/python

# import statements
import argparse
import serial
import os
import sys
import time
import signal
import threading

"""
exit handler
"""
def exit_handler(signum, frame):
    print "\nexiting!\n"
    s.close()
    sys.exit()

def waitKeyboard():
    if (sys.platform == "win32"):  # windows
        CLEAR_CMD = "cls"
    else:                 # some other OS (should insert more checks)
        CLEAR_CMD = "clear"

    while True:
        raw_input()
        os.system(CLEAR_CMD)


# Main code####################################################################
def main(args):

    # setup clearscreen thread
    t = threading.Thread(target=waitKeyboard)
    t.daemon = True
    t.start()



    # initialize the serial port
    # this may change, depending on what port the OS gives the microcontroller
    s.port = args.port
    s.baudrate = args.baudrate  # the baudrate may change in the future
    # attempt to open the serial port
    # (there is no guard code, I'm assuming this does not fail)
    s.open()

    signal.signal(signal.SIGINT, exit_handler)
    signal.signal(signal.SIGTERM, exit_handler)


    # Main reading loop
    while True:
        print hex(ord(s.read(1)))


# end main


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=str, required=True,
                        help="Serial port")
    parser.add_argument('-b', '--baudrate', type=int, default=9600,
                        help="Serial interface baudrate.")
    args = parser.parse_args()

    s = serial.Serial()    # get instance of serial class

    main(args)
