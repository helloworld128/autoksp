from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time
import multiprocessing as mp
from random import random


def runPlot(*queues):
    cnt = len(queues)
    x_data = [[] for _ in range(cnt)]
    y_data = [[] for _ in range(cnt)]
    figure, axes = plt.subplots(cnt, 1)
    for i, ax in enumerate(axes):
        ax.plot(x_data[i], y_data[i])

    def update(frame):
        lines = []
        for i, q in enumerate(queues):
            ax = axes[i]
            line, = ax.get_lines()
            while not q.empty():
                x, y = q.get()
                x_data[i].append(x)
                y_data[i].append(y)
            line.set_data(x_data[i], y_data[i])
            ax.relim()
            ax.autoscale_view()
            lines.append(line)
        return lines

    animation = FuncAnimation(figure, update, interval=200)
    plt.show()


def main():
    q1 = mp.Queue()
    q2 = mp.Queue()
    p = mp.Process(target=runPlot, args=(q1, q2))
    p.start()
    x = 0
    while 1:
        q1.put((x, np.sin(x)))
        q2.put((x, np.cos(x)))
        x += 0.01
        # print(f'{x=}')
        time.sleep(random() / 10)


if __name__ == '__main__':
    main()
