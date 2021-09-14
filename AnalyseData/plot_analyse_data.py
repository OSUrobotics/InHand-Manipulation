#!/usr/bin/env python3

import matplotlib.pyplot as plt
import analyse_data
import numpy as np


def get_data(df, find_in, strip_out):
    find_col = analyse_data.find_data_with_other_col(df, 'Phase', 'Move', find_in)
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
        p = plt.plot(x_data, y_data, color)
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
                                                  scale=data_to_plot[3], color=data_to_plot[5],track=track)[0])
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
    if 'dir_all' not in filename:
        set_axis_limits(dir)

    if save:
        if track:
            plt.savefig('TrackPlots/Even Better/{}'.format(filename))
        else:
            plt.savefig('Plots/Even Better/{}'.format(filename))
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
                                                                                scale_data_2*line2[y_data_2]], 'gray',
                 linestyle='dotted')


def set_axis_limits(dir):
    step_x = 0.005
    step_y = 0.001
    # limit =
    if dir == 'a':
        limits_x = np.arange(-0.006, 0.006, step_y)
        limits_y = np.arange(0, 0.05, step_x)
        plt.xlim([-0.006, 0.006])
    elif dir == 'b':
        limits_x = np.arange(0, 0.05, step_x)
        limits_y = np.arange(-0.01, 0.05, step_x)
        plt.xlim([0, 0.05])
        plt.ylim([-0.01, 0.05])
    elif dir == 'c':
        limits_x = np.arange(-0.008, 0.08, step_x)
        limits_y = np.arange(-0.006, 0.012, step_y)
        plt.xlim([-0.008, 0.08])
        plt.ylim([-0.006, 0.012])
    elif dir == 'd':
        limits_x = np.arange(0, 0.050, step_x)
        limits_y = np.arange(-0.04, 0.04, step_x)
        plt.xlim([0, 0.05])
        plt.ylim([-0.04, 0.04])
    elif dir == 'e':
        limits_x = np.arange(-0.002, 0.010, step_y)
        limits_y = np.arange(-0.030, 0.010, step_x)
        plt.xlim([-0.002, 0.010])
        plt.ylim([-0.03, 0.01])
    elif dir == 'f':
        limits_x = np.arange(-0.035, 0.0025, step_x)
        limits_y = np.arange(-0.05, 0.005, step_x)
        plt.xlim([-0.035, 0.0025])
        plt.ylim([-0.05, 0.01])
    elif dir == 'g':
        limits_x = np.arange(-0.05, 0.0, step_x)
        limits_y = np.arange(-0.002, 0.012, step_y)
    elif dir == 'h':
        limits_x = np.arange(-0.03, 0.0, step_x)
        limits_y = np.arange(0.0, 0.03, step_x)
    else:
        print("Wrong Direction!")
        raise ValueError
    _, labels_x = plt.xticks(limits_x)
    _, labels_y = plt.yticks(limits_y)
    plt.setp(labels_x, rotation=30, horizontalalignment='right')
    plt.setp(labels_y, rotation=30, horizontalalignment='right')

    return plt


if __name__ == '__main__':
    directory = '/Users/asar/PycharmProjects/InHand-Manipulation/AnalyseData/Data/Trial Data/Even Better'
    # direction = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    direction = ['b']
    subject = 'sub1'
    plot_type = 'single'
    # plot_type = 'combined_all'
    trial_hum = 'human_'+subject
    trial_exp = 'expected_' + 'exp'
    hand = 'new_hand'

    if plot_type == 'single':
        trial = trial_hum
        track = True
        save_trial = plot_type+'_plot_'+trial
    else:
        trial = ''
        track = False
        save_trial = plot_type+'_plots_'+subject
    dp = 1
    step = 7
    kp = None
    kd = None
    if len(direction) > 1:
        track = False
    for dir in direction:
        if dir == 'f' or dir == 'g':
            trial_num = 1
        else:
            trial_num = 1

        if 'single_plot' in save_trial:
            saved_data_file_name = directory+'/{}_{}_2v2_{}_n_{}_kp{}_kd{}_dp{}_step{}_save_data.csv'.format(
                hand, trial, dir, trial_num, kp, kd, dp, step)
            saved_df_controller = analyse_data.get_data(saved_data_file_name)
            human_data_col = get_data(saved_df_controller, find_in='human_cube_pos', strip_out='()')
            # print("HU: {}".format(human_data_col))
            # hello = plot_single_data(human_data_col, y=2, scale=1)
            # plt.show()
            controller_data_col = get_data(saved_df_controller, find_in='Cube_pos_in_start_pos', strip_out='[]')
            # print("CON: {}".format(controller_data_col))
            # hello = plot_single_data(controller_data_col, y=2, scale=1)
            # plt.show()
            plot1 = [human_data_col, 0, 2, 1, 'Human Trial', 'blue']
            plot2 = [controller_data_col, 0, 1, 1, 'Controller', 'red']
            all_plots = [plot1, plot2]
        else:
            saved_data_controller_from_human = directory+'/{}_{}_2v2_{}_n_{}_kp{}_kd{}_dp{}_step{}_save_data.csv'.\
                format(hand, trial_hum, dir, trial_num, kp, kd, dp, step)
            saved_data_controller_from_expected = directory+'/{}_{}_2v2_{}_n_{}_kp{}_kd{}_dp{}_step{}_save_data.csv'.\
                format(hand, trial_exp, dir, trial_num, kp, kd, dp, step)

            saved_df_controller_from_human = analyse_data.get_data(saved_data_controller_from_human)
            saved_df_controller_from_expected = analyse_data.get_data(saved_data_controller_from_expected)

            human_data_col = get_data(saved_df_controller_from_human, find_in='human_cube_pos', strip_out='()')
            # hello = plot_single_data(human_data_col, y=2, scale=1)
            # plt.show()

            human_controller_data_col = get_data(saved_df_controller_from_human, find_in='Cube_pos_in_start_pos', strip_out='[]')
            # plot_single_data(controller_data_col, y=1, scale=1)
            # plt.show()

            expected_controller_data_col = get_data(saved_df_controller_from_expected, find_in='Cube_pos_in_start_pos', strip_out='[]')
            # plot_single_data(controller_data_col, y=1, scale=1)
            # plt.show()

            dp_other = 1
            step_other = 1
            saved_data_controller_from_human_other = directory+'/{}_{}_2v2_{}_n_{}_kp{}_kd{}_dp{}_step{}_save_data.csv'\
                .format(hand, trial_hum, dir, trial_num, kp, kd, dp_other, step_other)
            saved_data_controller_from_expected_other = directory+'/{}_{}_2v2_{}_n_{}_kp{}_kd{}_dp{}_step{}_save_data.csv'.\
                format(hand, trial_exp, dir, trial_num, kp, kd, dp_other, step_other)

            saved_df_controller_from_human_other = analyse_data.get_data(saved_data_controller_from_human_other)
            saved_df_controller_from_expected_other = analyse_data.get_data(saved_data_controller_from_expected_other)

            human_controller_data_col_other = get_data(saved_df_controller_from_human_other, find_in='Cube_pos_in_start_pos', strip_out='[]')
            expected_controller_data_col_other = get_data(saved_df_controller_from_expected_other, find_in='Cube_pos_in_start_pos', strip_out='[]')

            plot1 = [human_data_col, 0, 2, 1, 'Human Trial', 'blue']
            plot2 = [human_controller_data_col, 0, 1, 1, 'Controller from Human', 'red']
            plot3 = [expected_controller_data_col, 0, 1, 1, 'Controller from Expected', 'gold']
            plot4 = [human_controller_data_col_other, 0, 1, 1, 'Controller from Human {} {}'.format(dp_other, step_other), 'm']
            plot5 = [expected_controller_data_col_other, 0, 1, 1, 'Controller from Expected {} {}'.format(dp_other, step_other), 'c']

            all_plots = [plot1, plot2, plot3, plot4, plot5]

        save_plot = True
        if len(direction) > 1:
            fig_name = '{}_dir_all_plotskp{}_kd{}_dp{}_steps{}.png'.format(save_trial, kp, kd, dp, step)
        else:
            fig_name = '{}_{}_all_plots_kp{}_kd{}_dp{}_steps{}.png'.format(save_trial, dir, kp, kd, dp, step)
        one_plot = [[human_data_col, 0, 2, 1, 'Human Trial']]
        plot_multiple_data(all_plots, labels=['X position in cms', 'Y position in cms'],
                           title='Movement of Cube in {}'.format(dir), dir=dir, save=save_plot, filename=fig_name, track=track)
