# Platform for Automated Vehicle Platooning
Truck platooning is a technology that enables two or more trucks to travel together in a coordinated way, using wireless communication and automated driving systems. The main benefit of this technology is to reduce fuel consumption and emissions, as well as to improve traffic flow and safety. The trucks form a close-following convoy, where the leading truck sets the speed and direction, and the following trucks automatically adjust their movements accordingly. The drivers remain in control of their vehicles at all times, and can choose to join or leave the platoon as they wish. Truck platooning is currently being tested and developed in various countries, with the aim of making it a viable option for long-distance freight transport in the future.

Some research has already been made in regards to platooning. In a [brief paper](https://www.sciencedirect.com/science/article/abs/pii/S0005109817301838) written by Ivo Herman, Steffi Knorn, and Anders Ahlén, a bidirectional system between the vehicles is looked at. For such a system, a vehicle does not only react to what the vehicle in front is doing, but also the vehicle behind. This approach has given inspiration to this project.

## Objectives
The objective is to autonomously drive vehicles in a platoon in an energy efficient way. Reducing the distance between vehicles in the platoon will result in less drag for following vehicles, and thus less energy being lost. However, the tradeoff is that reducing the distance between the vehicles too much can be unsafe in the case of disturbances, such as up or down slopes, traffic, etc.

This project uses a set of RC-vehicles that can are controlled with a Raspberry Pi 4 Model B, and uses sensors such as an ultrasonic sensor for distance measurement, a Pixy2 camera for tracking and following the vehicle in front, and a reflectance sensor for measuring the velocity.

## Usage
This project is based on ROS Noetic, which is only supported on Ubuntu 20.04 LTS. The Makefile contains all the required setup needed, and can be run using `make install`.

> The Makefile expects the user on the Raspberry Pi to be called `laff`, and the password to also be `laff`!

Once the installation is complete, you need to build the project using `make cat` (the tool that ROS uses to build source files is called catkin). If everything compiled as expected, you should now be able to run the software on the vehicle.

1. Ensure that the motor on the RC-vehicle is turned on (you should hear a beep after turning it on)
2. Train the Pixy2 camera if you have not already, preferrably by connecting to the camera with USB and using PixyMon.
2. Run one of the launch files in `src/rcv/launch/vehicle_*.launch`, depending on what you want to do. Names are self-explanatory.

The easiest way to use PixyMon is to either install it locally on your computer and connect it to the vehicle, or by running `make pixymon` and connecting to the Raspberry Pi using VNC at port 5900.

Most parameters of the vehicle is defined in `src/rcv/launch/shared.launch` and `src/rcv/launch/vehicle.launch`. Debug mode is enabled by default, and will write all important telemetry data to `/tmp/debug.csv`. Note that this file is not persistent between reboots - if needed, copy the file to a persistent location before rebooting.

## Orchestrator
The orchestrator allows you to control multiple vehicles at the same time directly from your own computer, without using individual SSH sessions. Start the orchestrator using `python -m orchestrator`, and the vehicles should automatically connect, assuming that they are connected to the same wireless network.

## Simulations
To run a simulation, do the following:
```bash
cd simulations
python simulation.py --ticks=14000 --scenario=4 --vehicles=21 --type=1 --model=5 --period=0.01`
```
Arguments can of course be adjusted as needed.