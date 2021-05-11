import matplotlib.pyplot as plt


def plot_human_data(data):
    x_data = []
    y_data = []
    scale = 0.1
    for line in data:
        x_data.append(scale*line[0])
        y_data.append(scale*line[1])
    # print("DATA 0: {},\n DATA 1: {},\n DATA: {}".format(data[:][0], data[1], data))
    plt.plot(x_data, y_data)
    plt.show()