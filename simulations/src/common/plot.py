import matplotlib.pyplot as plt
import numpy as np

def plot(data, num_vehicles, x, linestyle, alpha=1):
    for i in range(num_vehicles):
        # Extract the values from each sub-array
        y = np.array([sub_array[i] for sub_array in data])

        # Create a bar plot
        plt.plot(x, y, linestyle=linestyle, alpha=alpha)


def make_plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path):
    plt.clf()
    # Extract the number of elements in each sub-array
    x = np.arange(len(data))

    plot(data, num_vehicles, x, "-")

    plt.xlabel(x_label_text)
    plt.ylabel(y_label_text)
    plt.title(title_text)
    plt.savefig(download_path)

def plot_speed(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Speed (km/h)'
    title_text = 'Speed of vehicles'

    make_plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_travel_distance(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Position (m)'
    title_text = 'Travel distance of vehicles'

    make_plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_distances(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Distance (m)'
    title_text = 'Distance to vehicle in front'
    distances, references = data
    mean_references = np.mean(references, axis=1)
    mean_references_reshape = mean_references.reshape(-1, 1)
    mean_references = mean_references_reshape

    plt.clf()
    # Extract the number of elements in each sub-array
    x1 = np.arange(len(distances))
    x2 = np.arange(len(mean_references))

    plot(distances, num_vehicles, x1, linestyle="-")
    plot(mean_references, 1, x2, linestyle="-", alpha=0.4)

    plt.xlabel(x_label_text)
    plt.ylabel(y_label_text)
    plt.title(title_text)
    plt.savefig(download_path)

def plot_position(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Distance (m)'
    title_text = 'Relative position to the lead vehicle'

    make_plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)