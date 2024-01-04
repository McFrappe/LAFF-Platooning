leader = "v2"
follower_1 = "v1"
follower_2 = "v3"

period = 0.1

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read CSV file into a pandas DataFrame
df_f1 = pd.read_csv('csv-files/' + follower_1 + '.csv')
df_f2 = pd.read_csv('csv-files/' + follower_2 + '.csv')

#min_time_f1 = df_f1['time'].min()
#min_time_f2 = df_f2['time'].min()

# Extract time and distance columns
#time = df['time'] - min_time
distance_f1 = df_f1['distance']
distance_f2 = df_f2['distance']
velocity_f1 = df_f1['velocity']
velocity_f2 = df_f2['velocity']
time_f1 = np.linspace(0, np.size(distance_f1)*period, np.size(distance_f1))
time_f2 = np.linspace(0, np.size(distance_f2)*period, np.size(distance_f2))
time_v_f1 = np.linspace(0, np.size(velocity_f1)*period, np.size(velocity_f1))
time_v_f2 = np.linspace(0, np.size(velocity_f2)*period, np.size(velocity_f2))

# filter
def distance_filter(distance, initial):
    distance_filtered = np.zeros(np.size(distance))
    prev = initial #distance[0] # or initial value
    for i in range(np.size(distance)):
        curr = distance[i]
        change = min(max(curr - prev, -2), 0.01)
        distance_filtered[i] = min(max(prev + change, 0), 6)

        # prev should be defined last
        prev = distance_filtered[i]

    return distance_filtered

def distance_filter2(distance, initial):
    distance_filtered = np.zeros(np.size(distance))
    prev = initial #distance[0] # or initial value
    for i in range(np.size(distance)):
        curr = distance[i]
        diff = curr - prev
        #if diff > 0.1 or diff < -2 or curr < 0 or curr > 6:
        #if (diff > 0.1 or diff < -2 or curr < 0 or curr > 6) and (prev > 0 and prev < 6):
        if curr < 0 or curr > 4:
            distance_filtered[i] = prev
        else:
            distance_filtered[i] = curr

        # prev should be defined last
        prev = distance_filtered[i]

    return distance_filtered


def velocity_filter(velocity, initial):
    velocity_filtered = np.zeros(np.size(velocity))
    prev = initial
    max_decceleration_diff = 0.8
    max_acceleration_diff = 0.2
    for i in range(np.size(velocity)):
        curr = velocity[i]
        diff = curr-prev

        if curr == 0 and prev > max_decceleration_diff:
            velocity_filtered[i] = prev
        else:
            velocity_filtered[i] = prev + min(max(diff, -max_decceleration_diff), max_acceleration_diff)

        prev = velocity_filtered[i]

    return velocity_filtered


distance_f1_filtered = distance_filter(distance_f1, 0.2)
distance_f2_filtered = distance_filter(distance_f2, 0.2)

distance_f1_filtered2 = distance_filter2(distance_f1, 0.2)
distance_f2_filtered2 = distance_filter2(distance_f2, 0.2)

velocity_f1_filtered = velocity_filter(velocity_f1, 0.2)
velocity_f2_filtered = velocity_filter(velocity_f2, 0.2)

# Plot the data
def plot_distance():
    ## Distance 
    #plt.plot(time_f1, distance_f1, linestyle='dotted', label='Distance vs. Time f1')
    #plt.plot(time_f2, distance_f2, linestyle='dotted', label='Distance vs. Time f2')
    #plt.plot(time_f1, distance_f1_filtered, linestyle='-.', label='Distance filtered vs. Time f1')
    #plt.plot(time_f2, distance_f2_filtered, linestyle='-.', label='Distance filtered vs. Time f2')
    plt.plot(time_f1, distance_f1_filtered2, label='Distance filtered2 vs. Time f1')
    plt.plot(time_f2, distance_f2_filtered2, label='Distance filtered2 vs. Time f2')

    plt.xlabel('Time [100ms]')
    plt.ylabel('Distance [m]')
    plt.title('Distance vs. Time')
    plt.legend()
    plt.show()

def plot_velocity():
    #plt.plot(time_v_f1, velocity_f1, label='Velocity vs. Time f1')
    #plt.plot(time_v_f2, velocity_f2, label='Velocity vs. Time f2')
    plt.plot(time_v_f1, velocity_f1_filtered, label='Velocity filtered vs. Time f1')
    plt.plot(time_v_f2, velocity_f2_filtered, label='Velocity filtered vs. Time f2')

    plt.xlabel('Time [100ms]')
    plt.ylabel('Velocity [m]')
    plt.title('Velocity vs. Time')
    plt.legend()
    plt.show()

plot_distance()
#plot_velocity()
