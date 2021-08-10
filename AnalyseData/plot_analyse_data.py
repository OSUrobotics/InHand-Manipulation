#!/usr/bin/env python3

import matplotlib.pyplot as plt
import analyse_data
import numpy as np


def get_data(df, find_in, strip_out):
    find_col = analyse_data.find_data_with_other_col(saved_df, 'Phase', 'Move', find_in)
    find_col = find_col.fillna(find_col.loc[find_col.first_valid_index()])
    find_col_list = analyse_data.convert_string_list_to_list_float(find_col, strip_out)
    return find_col_list


def plot_single_data(df, x = 0, y = 1, scale=1.0):
    x_data = []
    y_data = []
    for line in df:
        x_data.append(scale*line[x])
        y_data.append(scale*line[y])
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    plt.plot(x_data, y_data)


def plot_expected_data(dir):
    """
    TODO: Fix this so that amount in direction reflects handspan
    :param dir:
    :return:
    """
    dist = 0.05
    start_at = 0.0
    if dir == 'a':
        plt.plot([0, 0], [0, 0.04])
    elif dir == 'b':
        plt.plot([0, 0.04],[0, 0.04])
    elif dir == 'c':
        plt.plot([0, dist], [0, 0])
    elif dir == 'd':
        plt.plot([0, 0.02], [0, -0.02])
    elif dir == 'e':
        plt.plot([0, 0], [0, -0.02])
    elif dir == 'f':
        plt.plot([0, -0.02], [0, -0.02])
    elif dir == 'g':
        plt.plot([0, -dist], [0, 0])
    elif dir == 'h':
        plt.plot([0, -0.03], [0, 0.03])
    else:
        print("Wrong Direction!")
        raise ValueError


def plot_multiple_data(data,labels, title, dir, save=False, filename=''):
    """
    Data should be a list of list of 5 elements:
    First 4 are the inputs of "plot_single_data" function. 5th element is the legend to be applied
    :param data:
    :param labels: list of label names for x and y, respectively
    :param title: Title of the plot
    :param dir: Direction of asterisk test data
    :return:
    """
    legend = []
    for data_to_plot in data:
        plot_single_data(df=data_to_plot[0], x=data_to_plot[1], y=data_to_plot[2], scale=data_to_plot[3])
        legend.append(data_to_plot[4])

    plot_expected_data(dir=dir)
    legend.append('Expected Trial')
    plt.legend(legend)
    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.title(title)

    if save:
        plt.savefig('Plots/{}'.format(filename))
    else:
        plt.show()


if __name__ == '__main__':
    direction = 'a'
    saved_data_file_name = '/Users/asar/PycharmProjects/InHand-Manipulation/AnalyseData/Data/filt_josh_2v2_{}_none_1_kpNone_kdNone_dp1_step1_save_data.csv'.format(direction) #'/Users/asar/PycharmProjects/InHand-Manipulation/AnalyseData/Data/filt_josh_2v2_g_none_1_' \
                           #'kpNone_kdNone_dp1_step1_save_data.csv'
    saved_df = analyse_data.get_data(saved_data_file_name)

    human_data_col = get_data(saved_df, find_in='human_cube_pos', strip_out='()')
    # hello = plot_single_data(human_data_col, y=2, scale=1)
    # plt.show()

    controller_data_col = get_data(saved_df, find_in='Cube_pos_in_start_pos', strip_out='[]')
    # plot_single_data(controller_data_col, y=1, scale=1)
    # plt.show()

    all_plots = [[human_data_col, 0, 2, 1, 'Human Trial'], [controller_data_col, 0, 1, 0.5, 'Controller Trial']]

    save_plot = True
    fig_name = '{}_dir_all_plots_expected.png'.format(direction)
    one_plot = [[human_data_col, 0, 2, 1, 'Human Trial']]
    plot_multiple_data(all_plots, labels=['X position in cms', 'Y position in cms'], title='Movement of Cube in {}'.format(direction), dir=direction, save=save_plot, filename=fig_name)
