import matplotlib.pyplot as plt


def plot_human_data(data):
    x_data = []
    y_data = []
    scale = 0.1
    for line in data:
        x_data.append(scale*line[3])
        y_data.append(scale*line[4])
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    plt.plot(x_data, y_data)
    plt.show()


def plot_human_and_controller_data(human_data, controller_data):
    """
    Plot the human study data and controller data side by side.
    :param human_data: x,y position of data from human study
    :param controller_data: x,y position of expert controller
    :return:
    """
    x_data_human = []
    y_data_human = []

    x_data_controller = []
    y_data_controller = []

    scale = 0.1
    for line in human_data:
        x_data_human.append(scale*line[3])
        y_data_human.append(scale*line[4])
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    plt.plot(x_data_human, y_data_human)

    # y_data_first_controller = controller_data[0][0][0]
    # x_data_first_controller = controller_data[0][0][1]
    x_data_first_controller = controller_data[0][0][0]
    y_data_first_controller = controller_data[0][0][1]
    for line in controller_data:
        # print("LINE: {} {}".format(line[0][0], line[0][1]))
        x_data_controller.append(line[0][0] - x_data_first_controller)
        y_data_controller.append(line[0][1] - y_data_first_controller)
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    plt.plot(x_data_controller, y_data_controller)

    plt.show()
