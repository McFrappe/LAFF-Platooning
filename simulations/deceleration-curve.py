def deceleration(t, v_0):
    b_r = -1 # break coefficent
    return b_r*t + v_0 # the velocity


def deceleration_with_ref(ref):
    return deceleration(1, ref)

def v_with_sampeling(t, t_c, v_old, v_new):
    if t % t_c == 0:
        return v_new
    else:
        return v_old

def simulate():
    v_leader = 100 # v_0
    v_0 = v_leader
    v_old_leader = v_leader
    v_follower = v_leader # v_0
    t = 0
    t_c = 5

    while v_leader > 0:
        t = 1+t
        if t % t_c == 0:
            v_old_leader = v_leader
        v_leader = deceleration(t, v_0)

        # you have the target sampeling with 
        # this is wrong with 
        v_follower_ref = v_with_sampeling(t, t_c, v_old_leader, v_leader)
        v_follower = deceleration_with_ref(v_follower_ref)
        print(v_leader, v_follower)


if __name__ == "__main__":
    simulate()
