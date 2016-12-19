import numpy as np
import matplotlib.pyplot as plt
import argparse
import rospy
import subprocess
import importlib
import signal

def sig_handler(signum, frame):
    global h
    h.plot()

class Histogram():

    def __init__(self, arguments):
        self.args = arguments

        # Determine type of topic we are to listen to
        output = subprocess.Popen(['rostopic', 'type', self.args.topic],
                                  stdout=subprocess.PIPE).communicate()[0]
        type = output.split('/')
        i = importlib.import_module(type[0] + ".msg", type[1])

        self.sub = rospy.Subscriber(self.args.topic, getattr(i, type[1][:-1]),
                                    callback=self.callback)
        print "Subscribed to {}".format(self.args.topic)
        self.data = []

    def callback(self, msg):
        print "Got Message"
        self.data.append(eval('msg.' + self.args.attribute))

    def plot(self):
        n, bins, patches = plt.hist(self.data, 'auto', normed=self.args.normed,
                                    facecolor=self.args.color,
                                    alpha=self.args.alpha)

        if self.args.normed:
            plt.ylabel('Probability')
        else:
            plt.ylabel('Number')

        plt.xlabel('Value')
        plt.title('Histogram of {}'.format(args.topic))

        plt.show()

if __name__ == "__main__":
    global h

    signal.signal(signal.SIGINT, sig_handler)

    rospy.init_node("Histogram", anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('topic', help="Topic to plot data from")
    parser.add_argument('attribute', help="Attribute of the message to plot")
    parser.add_argument('--normed', action='store_true', default=False,
                        help="Normalize the data in the Histogram")
    parser.add_argument('--color', default='blue',
                        help="Choose the color of the bars in the plot")
    parser.add_argument('--alpha', type=float, default=0.75,
                        help="Alpha of the color to use")
    args = parser.parse_args()

    h = Histogram(args)

    rospy.spin()
