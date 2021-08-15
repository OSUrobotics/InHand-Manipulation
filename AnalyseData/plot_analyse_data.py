#!/usr/bin/env python3

import matplotlib.pyplot as plt
import analyse_data
import numpy as np


def get_data(df, find_in, strip_out):
    find_col = analyse_data.find_data_with_other_col(saved_df, 'Phase', 'Move', find_in)
    find_col = find_col.fillna(find_col.loc[find_col.first_valid_index()])
    find_col_list = analyse_data.convert_string_list_to_list_float(find_col, strip_out)
    return find_col_list


def plot_expected_data(dir):
    """
    TODO: Fix this so that amount in direction reflects handspan
    :param dir:
    :return:
    """
    dist = 0.05
    start_at = 0.0
    color = 'green'
    if dir == 'a':
        return plt.plot([0, 0], [0, 0.04], color)
    elif dir == 'b':
        return plt.plot([0, 0.04], [0, 0.04], color)
    elif dir == 'c':
        return plt.plot([0, dist], [0, 0], color)
    elif dir == 'd':
        return plt.plot([0, 0.02], [0, -0.02], color)
    elif dir == 'e':
        return plt.plot([0, 0], [0, -0.02], color)
    elif dir == 'f':
        return plt.plot([0, -0.02], [0, -0.02], color)
    elif dir == 'g':
        return plt.plot([0, -dist], [0, 0], color)
    elif dir == 'h':
        return plt.plot([0, -0.03], [0, 0.03], color)
    else:
        print("Wrong Direction!")
        raise ValueError


def plot_single_data(df, x=0, y=1, scale=1.0, color='red', track=True):
    x_data = []
    y_data = []
    for line in df:
        x_data.append(scale * line[x])
        y_data.append(scale * line[y])
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    if track:
        p = plt.scatter(x_data, y_data, c=color)
    else:
        p = plt.plot(x_data, y_data, color)
    return p


def plot_multiple_data(data, labels, title, dir, save=False, filename='', track=True):
    """
    Data should be a list of list of 5 elements:
    First 4 are the inputs of "plot_single_data" function. 5th element is the legend to be applied
    :param data:
    :param labels: list of label names for x and y, respectively
    :param title: Title of the plot
    :param dir: Direction of asterisk test data
    :return:
    """
    legends = []
    plot_pointers = []
    for data_to_plot in data:
        if track:
            plot_pointers.append(plot_single_data(df=data_to_plot[0], x=data_to_plot[1], y=data_to_plot[2],
                                                  scale=data_to_plot[3], color=data_to_plot[5],track=track))
        else:
            plot_pointers.append(plot_single_data(df=data_to_plot[0], x=data_to_plot[1], y=data_to_plot[2],
                                                  scale=data_to_plot[3], color=data_to_plot[5],track=track)[0])
        legends.append(data_to_plot[4])
        # plt.legend([data_to_plot[4]])

    plot_pointers.append(plot_expected_data(dir=dir)[0])

    legends.append('Expected Trial')
    # print("{}\n\n{}".format(plot_pointers, legends))
    plt.legend(plot_pointers, legends)
    if track:
        plot_track_human_and_controller(data)

    plt.xlabel(labels[0])
    plt.ylabel(labels[1])
    plt.title(title)

    if save:
        if track:
            plt.savefig('TrackPlots/{}'.format(filename))
        else:
            plt.savefig('Plots/{}'.format(filename))
    else:
        plt.show()


def plot_track_human_and_controller(data):
    # for data_to_plot in data:
    #     if data_to_plot[4] == 'Human Trial'
    #         scale
    data1 = list(data[0])
    x_data_1 = data1[1]
    y_data_1 = data1[2]
    scale_data_1 = data1[3]
    data2 = list(data[1])
    x_data_2 = data2[1]
    y_data_2 = data2[2]
    scale_data_2 = data2[3]
    # print(x_data_1, y_data_1, scale_data_1, x_data_2, y_data_2, scale_data_2)
    # print("HELLO human: {}".format(list(data[0])))
    # print("HELLO controller: {}".format(list(data[1])))

    for line1, line2 in zip(list(data[0][0]), list(data[1][0])):
        # print("HELLO", line1, line2)
        # print("HELLO: {}, I: ".format(line.values))
        # print("Scale: {} i: {}, ".format(line.values, i))
        plt.plot([scale_data_1*line1[x_data_1], scale_data_2*line2[x_data_2]], [scale_data_1*line1[y_data_1],
                                                                                scale_data_2*line2[y_data_2]])
        pass
    pass


if __name__ == '__main__':
    direction = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    # direction = ['h']
    # trial = 'human_filt_josh'
    trial = 'expected_exp'
    hand = 'new_hand'
    track=False
    dp = 5
    step = 5
    kp = None
    kd = None
    for dir in direction:
        if dir == 'f' or dir == 'g':
            trial_num = 1
        else:
            trial_num = 1

        saved_data_file_name = '/Users/asar/PycharmProjects/InHand-Manipulation/AnalyseData/Data/Trial Data/{}_{}_2v2_{}_none_{}_kp{}_kd{}_dp{}_step{}_save_data.csv'.format(
            hand, trial, dir, trial_num, kp, kd, dp, step)

        saved_df = analyse_data.get_data(saved_data_file_name)

        human_data_col = get_data(saved_df, find_in='human_cube_pos', strip_out='()')
        # hello = plot_single_data(human_data_col, y=2, scale=1)
        # plt.show()

        controller_data_col = get_data(saved_df, find_in='Cube_pos_in_start_pos', strip_out='[]')
        # plot_single_data(controller_data_col, y=1, scale=1)
        # plt.show()
        plot1 = [human_data_col, 0, 2, 1, 'Human Trial', 'blue']
        plot2 = [controller_data_col, 0, 1, 0.5, 'Controller Trial', 'red']

        all_plots = [plot1, plot2]

        save_plot = True
        if len(direction) > 1:
            fig_name = '{}_dir_all_plotskp{}_kd{}_dp{}_steps{}.png'.format(trial, kp, kd, dp, step)
        else:
            fig_name = '{}_{}_all_plots_kp{}_kd{}_dp{}_steps{}.png'.format(trial, dir, kp, kd, dp, step)
        one_plot = [[human_data_col, 0, 2, 1, 'Human Trial']]
        plot_multiple_data(all_plots, labels=['X position in cms', 'Y position in cms'],
                           title='Movement of Cube in {}'.format(dir), dir=dir, save=save_plot, filename=fig_name, track=track)
