from platoon import Platoon

def simulate(num_steps):
    p = Platoon(2)  # all vehicles are standing still in an imaginary position of 0

    for s in range(num_steps):
        print(s)
        
        speeds, positions, distances = p.run(s)  # should use data that is returned and plot it 

        print(f"Speed: {speeds}")
        print(f"Position: {positions}")
        print(f"Distance: {distances}")


if __name__ == "__main__":
    simulate(100)