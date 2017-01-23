#!/usr/bin/env python2

from math import *
import random

landmarks = [[20.0, 20.0], [80.0, 80.0], [20.0, 80.0], [80.0, 20.0]]
world_size = 100.0

class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')

        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')

        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError('Orientation must be in [0..2pi]')

        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_noise = float(new_s_noise)

    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + i
                        (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)

        return Z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError("Robot cant move backwards")

        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + \
                      random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size  # cyclic truncate
        y %= world_size
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and
        # var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / \
               sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        # calculates how likely a measurement should be
        prob = 1.0
        ideal = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 +
                        (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])

        return prob

    def __repr__(self):
        return ("[x={0} y={1} orient={2}]".format('{:.5}'.format(str(self.x)),
           '{:.5}'.format(str(self.y)), '{:.5}'.format(str(self.orientation))))

# evaluation algorithm that gets the overall error from
# the list of particles after being resampled
# LOWER IS BETTER!
def rEval(r, p):
    sum = 0.0
    for i in range(len(p)):  # calculate mean error
        dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
        dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
        err = sqrt(dx * dx + dy * dy)
        sum += err

    return sum / float(len(p))

# ----- code
N = 1000
T = 10

# TODO: set variables for noise
# TODO: mimic sub with robot
# TODO: Read real data
# TODO: change sensor noise to also mimic real sub sensors


def run():
    RoboSub = robot()

    # ACTUAL PARTICLE FILTER
    # create list of N particles that are randomly instantiated
    p = []
    for i in range(N):
        r = robot()
        r.set_noise(.05, .05, 5.0)
        p.append(r)


    # For amount of iterations provided previously
    for t in range(T):
        # get the sense data from the robot
        z = RoboSub.sense()

        # append the weights from each of the particles
        # to a weighted list
        w = []
        for i in range(N):
            w.append(p[i].measurement_prob(z))

        # create a new list to add the particles
        # get the max weight from the list and start
        # at random index so we are not biased by weights
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)

        # We must reweight, so lets just allow
        # an iteration per particle. Could use more,
        # could use less, doesn't matter, just the N
        # is a nice number to use and is readily available
        # and has a meaning
        for i in range(N):
            # get a random weight based on the maxweight (range (0, 2*mw)
            beta += random.random() * 2.0 * mw

            # -while the random weight is larger than the index
            # this helps us parse out smaller particles, since they will
            # almost always be less
            while beta > w[index]:
                # we subtract the weight to make it more likely
                # to add the next particles
                beta -= w[index]
                # move to the next index, % for a circular motion
                index = (index + 1) % N

            # add a partilce from the previous index,
            # higher weighted partices are more likely to be readded
            p3.append(p[index])

        # set the original list to the new resampled list
        p = p3

        # print the evaluation of how strong the overall list is
        print(rEval(RoboSub, p))

    for i in range(N):
        print("#" + str(i) + ": " + str(p[i]))

    print("R: " + str(RoboSub))

def garbage():
    x = random.random()
    y = random.random()
    z = random.random()

    return [x, y, z]
