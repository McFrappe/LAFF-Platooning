import matplotlib.pyplot as plt
import numpy as np

def plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path):
    plt.clf()
    # Extract the number of elements in each sub-array
    x = np.arange(len(data))

    for i in range(num_vehicles):
        # Extract the values from each sub-array
        y = np.array([sub_array[i] for sub_array in data])

        # Create a bar plot
        plt.plot(x, y, linestyle="-")

    plt.xlabel(x_label_text)
    plt.ylabel(y_label_text)
    plt.title(title_text)
    plt.savefig(download_path)

def plot_speed(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Speed (km/h)'
    title_text = 'Speed of vehicles'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_travel_distance(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Position (m)'
    title_text = 'Travel distance of vehicles'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_distances(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Distance (m)'
    title_text = 'Distance to vehicle in front'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_position(data, num_vehicles, download_path):
    x_label_text = 'Ticks (10ms)'
    y_label_text = 'Distance (m)'
    title_text = 'Relative position to the lead vehicle'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)