from platoon import Platoon
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

def plot_speed(data, num_vehicles):
    x_label_text = 'Steps (h)'
    y_label_text = 'Speed (km/h)'
    title_text = 'Speed of vehicles'
    download_path = 'plots/speeds-with-distance-model.png'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_position(data, num_vehicles):
    x_label_text = 'Steps (h)'
    y_label_text = 'Position (km)'
    title_text = 'Position of vehicles'
    download_path = 'plots/position-with-distance-model.png'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def plot_distances(data, num_vehicles):
    x_label_text = 'Steps (h)'
    y_label_text = 'Distance (km)'
    title_text = 'Distance to vehicle in front'
    download_path = 'plots/distances-with-distance-model.png'

    plot(data, num_vehicles, x_label_text, y_label_text, title_text, download_path)

def simulate(num_steps, num_vehicles):
    p = Platoon(num_vehicles)  # all vehicles are standing still in an imaginary position of 0

    for s in range(num_steps):
        p.run(s)

    speeds = p.get_speeds()
    positions = p.get_positions()
    distances = p.get_distances()

    plot_speed(speeds, num_vehicles)
    plot_position(positions, num_vehicles)
    plot_distances(distances, num_vehicles)

if __name__ == "__main__":
    simulate(100, 8)