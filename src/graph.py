#!/usr/bin/env python3

"""
Plot the output of *_receive_node

Usage:
ros2 run pipe_perf time_receive_node > perf.txt
^C after a while
python3 graph.py
"""
import sys

import matplotlib.pyplot as plt


def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'perf.txt'

    count = 0
    xs = []
    means = []
    # stdevs = []
    lows = []
    highs = []
    f = open(filename)

    # Skip the first few lines
    for i in range(0, 10):
        f.readline()

    while True:
        line = f.readline()

        if not line:
            break

        words = line.split(' ')
        if len(words) < 6:
            break

        # Drop any crazy outliers
        mean = int(words[5])
        stdev = int(words[8])
        if mean > 10000 or mean < -10000 or stdev > 10000:
            print('drop: ', line)
            continue

        count += 1
        xs.append(count)
        means.append(mean)
        # stdevs.append(stdev)
        lows.append(mean - stdev)
        highs.append(mean + stdev)

    # plt.errorbar(xs, means, yerr=stdevs, alpha=0.8, elinewidth=1)
    plt.plot(xs, means)
    plt.fill_between(xs, lows, highs, color='gray', alpha=0.2)
    plt.show()


if __name__ == '__main__':
    main()
