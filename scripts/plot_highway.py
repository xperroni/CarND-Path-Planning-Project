#!/usr/bin/env python

from matplotlib import pyplot as pp


def plot(path):
    points = [tuple(float(s) for s in line.split(' ')[:2]) for line in open(path)]
    (x, y) = zip(*points)
    pp.plot(x, y, 'b.')

    pp.show()


def main():
    from sys import argv
    plot(argv[1])


if __name__ == '__main__':
    main()
