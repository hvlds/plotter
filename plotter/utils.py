import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def save_graph(x_values, y_values, x_label, y_label, output_file):
    x = np.array(x_values)
    y = np.array(y_values)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.plot(x, y, ".-")
    plt.savefig(output_file, bbox_inches="tight")
    plt.clf()

def plot_comparison(apriltag_s3_csv, apriltag_s2_csv, infrared_csv, base, output):
    apriltag_s3 = pd.read_csv(apriltag_s3_csv)
    apriltag_s2 = pd.read_csv(apriltag_s2_csv)
    infrared = pd.read_csv(infrared_csv)

    components = ["x", "y", "z"]
    for component in components:
        apriltag_time = apriltag_s3["time"]
        apriltag_comp = apriltag_s3[component]
        plt.plot(apriltag_time, apriltag_comp, label = "apriltag_s3")

        apriltag_time = apriltag_s2["time"]
        apriltag_comp = apriltag_s2[component]
        plt.plot(apriltag_time, apriltag_comp, label = "apriltag_s2")

        infrared_time = infrared["time"]
        infrared_comp = infrared[component]
        plt.plot(infrared_time, infrared_comp, label = "infrared")

        if component == "z":
            plt.axhline(y=base, color='r', linestyle='-', label="Groundtruth")

        plt.xlabel("Time [s]")
        plt.ylabel(f"Position in {component} [cm]")

        plt.legend()
        plt.savefig(f"{output}_{component}.png", bbox_inches="tight")

        plt.clf()


if __name__ == "__main__":
    pass