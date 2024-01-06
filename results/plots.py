period = 0.1

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Plot the data
def plot_distance(l, f1, f2, name_run):
    time_l = np.linspace(0, np.size(l)*period, np.size(l))
    time_f1 = np.linspace(0, np.size(f1)*period, np.size(f1))
    time_f2 = np.linspace(0, np.size(f2)*period, np.size(f2))

    plt.clf()

    plt.plot(time_l, l, label='Leader')
    plt.plot(time_f1, f1, label='Follower 1')
    plt.plot(time_f2, f2, label='Follower 2')

    plt.xlabel('Time [100ms]')
    plt.ylabel('Distance [m]')
    plt.title(f'Distance {name_run}')
    plt.legend()
    plt.savefig(f'plots/distance/distance_{name_run}')

def plot_velocity(l, f1, f2, name_run):
    time_l = np.linspace(0, np.size(l)*period, np.size(l))
    time_f1 = np.linspace(0, np.size(f1)*period, np.size(f1))
    time_f2 = np.linspace(0, np.size(f2)*period, np.size(f2))

    plt.clf()

    plt.plot(time_l, l, label='Leader')
    plt.plot(time_f1, f1, label='Follower 1')
    plt.plot(time_f2, f2, label='Follower 2')

    plt.xlabel('Time [100ms]')
    plt.ylabel('Velocity [km/h]')
    plt.title(f'Velocity {name_run}')
    plt.legend()
    plt.savefig(f'plots/velocity/velocity_{name_run}')


def plot_target_center_offset(l, f1, f2, name_run):
    time_l = np.linspace(0, np.size(l)*period, np.size(l))
    time_f1 = np.linspace(0, np.size(f1)*period, np.size(f1))
    time_f2 = np.linspace(0, np.size(f2)*period, np.size(f2))

    plt.clf()

    plt.plot(time_l, l, label='Leader')
    plt.plot(time_f1, f1, label='Follower 1')
    plt.plot(time_f2, f2, label='Follower 2')

    plt.xlabel('Time [100ms]')
    plt.ylabel('Offset [pixels]')
    plt.title(f'Target center offset {name_run}')
    plt.legend()
    plt.savefig(f'plots/target_center_offset/target_center_offset_{name_run}')


def plot(name_run):
    leader = "master"
    follower1 = "middle"
    follower2 = "last"
    path = 'csv-files/experiments_PID/'

    df_l = pd.read_csv(path + name_run + '/' + leader + '.csv')
    df_f1 = pd.read_csv(path + name_run + '/' + follower1 + '.csv')
    df_f2 = pd.read_csv(path + name_run + '/' + follower2 + '.csv')

    plot_distance(df_l['distance'], df_f1['distance'], df_f2['distance'], name_run)
    plot_velocity(df_l['velocity'], df_f1['velocity'], df_f2['velocity'], name_run)
    plot_target_center_offset(df_l['target_center_offset'], df_f1['target_center_offset'], df_f2['target_center_offset'], name_run)



plot('freerun')
plot('straight')
plot('take2_straight')
plot('take2_turn180')
plot('take3_turn180')
plot('turn180')
plot('turn90')