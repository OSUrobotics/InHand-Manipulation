#!/usr/bin/env python3

import numpy as np
import math
import random
import matplotlib.pyplot as plt


if __name__ == '__main__':
    some_num = 100000
    save_data = np.zeros(some_num)
    for i in range(0, some_num):
        x = math.floor(10 ** random.random())
        # x = np.random.randint(1, 10)
        save_data[i] = x
    plt.hist(save_data)
    plt.show()
    print(save_data.shape)
