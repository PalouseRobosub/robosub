import numpy as np
import matplotlib.pyplot as plt
import argparse
import rospy
import signal
import rostopic

def sig_handler(signum, frame):
    global h
    if h is not None:
        h.plot()

class Histogram():
    def __init__(self, arguments):
        self.args = arguments

        # Get the class type of the requested topic, blocking until it exists
        t_type = rostopic.get_topic_class(self.args.topic, blocking=True)

        # Subscribe to the topic
        self.sub = rospy.Subscriber(self.args.topic, t_type[0],
                                    callback=self.callback)

        rospy.loginfo("Subscribed to {}".format(self.args.topic))
        self.data = []

    def callback(self, msg):
        rospy.logdebug("Got {} Message".format(self.args.topic.split('/')[:-1]))
        self.data.append(eval('msg.' + self.args.attribute))

    def plot(self):
        try:
            n, bins, patches = plt.hist(self.data, int(self.args.bins),
                                        normed=1, facecolor=self.args.color,
                                        alpha=self.args.alpha)
        except ValueError:
            n, bins, patches = plt.hist(self.data, self.args.bins, normed=1,
                                        facecolor=self.args.color,
                                        alpha=self.args.alpha)

        if self.args.normed:
            plt.ylabel('Probability')
        else:
            plt.ylabel('Number')

        plt.xlabel('Value')
        plt.title('Histogram of {}'.format(args.topic))

        plt.show()

    def __del__(self):
        self.sub.unregister()

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
    parser.add_argument('--bins', default='auto',
                        help="Number of bins to use for the histogram")
    args = parser.parse_args()

    h = Histogram(args)

    rospy.spin()
