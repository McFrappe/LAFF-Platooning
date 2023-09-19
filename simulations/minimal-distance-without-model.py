import numpy as np
import matplotlib.pyplot as plt

def get_velocity_of_leader_for_follower(position, t_c, v_old_leader, v_new_leader):
    # t_c is the period
    if position % t_c == 0:
        return v_new_leader, True
    else:
        return v_old_leader, False

# This should be extended later on
def velocity_for_follower(d_min, d_to_leader, v_current, v_leader):
    if d_to_leader < d_min:
        print("Your to close to the leader")

    # Very simple model to decide speed
    return v_current + (v_leader - v_current) 


# Should instead be random but now the leader increase until the mid point and then decrease.
def velocity_for_leader(current_position, v_current, d_start_decreasing_v):
    if current_position < d_start_decreasing_v:
        return v_current+1
    else:
        return v_current-1



def simulate(num_steps):
    # Currently only for 2 cars
    # v_a, v_b
    # d_ab
    # v = d / step
    # will simulate d_end steps, i.e., one step is 1 (m)

    v_f = 0
    v_l = 0
    v_old_leader = v_l

    d_follower_initial = 4
    d_min = 9 
    d_to_leader = -1

    d_leader = 0 # the position of the leader
    d_follower = 0 # the position of the leader
    d_start_decreasing_v = ((num_steps/2)*(num_steps/2))/2 # increase until mid point then decrease

    arr = np.array([0])

    for s in range(num_steps):
        v_l = velocity_for_leader(d_leader, v_l, d_start_decreasing_v)
        d_leader = d_leader + v_l # since v_l is d per step

        if d_leader >= d_follower_initial:
            v_leader, new_value = get_velocity_of_leader_for_follower(s, 10, v_old_leader, v_l)

            if new_value:
                v_old_leader = v_leader

            d_to_leader = d_leader - d_follower # approximated since d is constant and does not depend on v_leader 
            v_f = velocity_for_follower(d_min, d_to_leader, v_f, v_leader)
            d_follower = d_follower + v_f

        arr = np.append(arr, d_to_leader)
        print(f"speed: {v_l}, {v_f}")
        print(f"distance between leader and follower: {d_to_leader}")


    # Generate x-axis values as the indices of the elements
    x = np.arange(len(arr))

    # Create a bar plot (you can also use plt.plot for a line plot)
    plt.bar(x, arr)

    # Add labels and a title to the plot
    plt.xlabel('step')
    plt.ylabel('distance')
    plt.title('Distance between leader and follower')

    # Show the plot
    plt.savefig('distance_between_leader_and_follower.png')


if __name__ == "__main__":
    simulate(1000)