#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import argparse
import rospy
import rostopic

class Histogram():
    def __init__(self, arguments):
        self.args = arguments

        self.data = {}

        self.labels = []

        for i in range(0,len(self.args.topics)):

            # Get the class type of the requested topic, blocking until it exists
            t_type = rostopic.get_topic_class(self.args.topics[i], blocking=True)

            # Subscribe to the topic
            self.sub = rospy.Subscriber(self.args.topics[i], t_type[0],
                                        callback=self.callback, callback_args=self.args.topics[i])

            rospy.loginfo("Subscribed to {}".format(self.args.topics[i]))

            self.data[self.args.topics[i]] = {}

            self.data[self.args.topics[i]]['data'] = []
            self.data[self.args.topics[i]]['attr'] = self.args.attributes[i]

            self.labels.append(self.args.topics[i] + '/' + self.args.attributes[i])

        plt.ion()
        plt.show()

    def callback(self, msg, topic):
        rospy.logdebug("Got {} Message".format(topic))
        self.data[topic]['data'].append(eval('msg.' + self.data[topic]['attr']))

    def plot(self):
        rospy.logdebug("Plotting Update")
        plt_data = []
        for value in self.data.values():
            plt_data.append(value['data'])

        plt_data = np.array(plt_data).T

        try:
            n, bins, patches = plt.hist(plt_data, int(self.args.bins),
                                        color=['blue', 'red'], histtype='bar',
                                        normed=1,
                                        alpha=self.args.alpha,
                                        label=self.labels)
        except ValueError:
            n, bins, patches = plt.hist(plt_data, self.args.bins, normed=1,
                                        color=['blue', 'red'], histtype='bar',
                                        alpha=self.args.alpha,
                                        label=self.labels
                                        )

        if self.args.normed:
            plt.ylabel('Probability')
        else:
            plt.ylabel('Number')

        plt.xlabel('Value')
        plt.title('Histogram')

        plt.legend()

        plt.pause(0.05)
        plt.gca().clear()

    def __del__(self):
        self.sub.unregister()

def timerCallback(timerEvent):
    global h
    rospy.logdebug("Timer Triggered")
    h.plot()

if __name__ == "__main__":
    global h

    rospy.init_node("Histogram", anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--topics', nargs='+', help="Topic to plot data from")
    parser.add_argument('--attributes', nargs='+', help="Attribute of the message to plot")
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

    rospy.Timer(rospy.Duration(1), timerCallback)

    rospy.spin()
