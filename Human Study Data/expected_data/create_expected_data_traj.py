#!/usr/bin/env python3

import csv
import numpy as np


def create_traj(dir, stop_at=50, start_at=0.0, samples=80):
    """
    Create straight in given directin
    :param dir: alphabet signalling the direction of motion
    :return: data: array of straight line traj
    """
    # stop_at = 0.5
    # start_at = 0.0
    # samples = 80

    if dir == 'a':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                           stop=[start_at, stop_at, start_at, start_at, stop_at, start_at], num=samples)
    elif dir == 'b':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                           stop=[stop_at, stop_at, start_at, stop_at, stop_at, start_at], num=samples)
    elif dir == 'c':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                           stop=[stop_at, start_at, start_at, stop_at, start_at, start_at], num=samples)
    elif dir == 'd':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                       stop=[stop_at, -stop_at, start_at, stop_at, -stop_at, start_at], num=samples)
    elif dir == 'e':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                           stop=[start_at, -stop_at, start_at, start_at, -stop_at, start_at], num=samples)
    elif dir == 'f':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                           stop=[-stop_at, -stop_at, start_at, -stop_at, -stop_at, start_at], num=samples)
    elif dir == 'g':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                       stop=[-stop_at, start_at, start_at, -stop_at, start_at, start_at], num=samples)
    elif dir == 'h':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                       stop=[-stop_at, stop_at, start_at, -stop_at, stop_at, start_at], num=samples)
    else:
        data = None

    index = np.arange(0, samples, dtype=int).T
    index = index.reshape(samples,1)
    data = np.concatenate((index, data), axis=1)
    # print("Data", data)
    return data


def save_traj(file_name, data_to_save):
    """
    Save data in csv format
    :param file_name: Name of file to save
    :param data_to_save: numpy array of traj
    :return:
    """
    first_row = ['frame' ,'x','y','rmag','f_x','f_y','f_rot_mag']
    with open(file_name, 'w') as f:
        write_it = csv.writer(f, delimiter=',')
        write_it.writerow(first_row)
        for line in data_to_save:
            write_it.writerow(line)


if __name__ == '__main__':
    direction = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    for alph in direction:
        filename = '/Users/asar/PycharmProjects/InHand-Manipulation/Human Study Data/expected_data/exp_2v2_{}_n_1.csv'.format(alph)
        if alph == 'c' or alph =='d' or alph == 'f' or alph == 'g':
            stop = 80
        else:
            stop = 50
        created_traj = create_traj(dir=alph, stop_at=stop)
        save_traj(filename, created_traj)
