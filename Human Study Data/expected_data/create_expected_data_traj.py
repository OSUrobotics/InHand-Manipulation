#!/usr/bin/env python3

import csv
import numpy as np


def create_traj(dir):
    """
    Create straight in given directin
    :param dir: alphabet signalling the direction of motion
    :return: data: array of straight line traj
    """
    stop_at = 4.0
    start_at = 0.0
    samples = 120

    if dir == 'g':
        data = np.linspace(start=[start_at, start_at, start_at, start_at, start_at, start_at],
                           stop=[-stop_at, start_at, start_at, -stop_at, start_at, start_at], num=samples)

    else:
        data = None

    index = np.arange(0, samples, dtype=int).T
    index = index.reshape(samples,1)
    data = np.concatenate((index, data), axis=1)
    return data


def save_traj(file_name, data_to_save):
    """
    Save data in csv format
    :param file_name: Name of file to save
    :param data_to_save: numpy array of traj
    :return:
    """
    first_row = ['' ,'x','y','rmag','f_x','f_y','f_rot_mag']
    with open(file_name, 'w') as f:
        write_it = csv.writer(f, delimiter=',')
        write_it.writerow(first_row)
        for line in data_to_save:
            write_it.writerow(line)


if __name__ == '__main__':
    alph = 'g'
    filename = '/Users/asar/PycharmProjects/InHand-Manipulation/Human Study Data/expected_data/exp_2v2_g_none_1.csv'
    created_traj = create_traj(dir=alph)
    save_traj(filename, created_traj)
