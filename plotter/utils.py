import matplotlib.pyplot as plt
import numpy as np


def save_graph(x_values, y_values, x_label, y_label, output_file):
    x = np.array(x_values)
    y = np.array(y_values)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.plot(x, y, ".-")
    plt.savefig(output_file, bbox_inches="tight")
    plt.clf()
