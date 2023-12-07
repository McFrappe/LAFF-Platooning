# LAFF-Platooning
Open Advanced Course in Embedded Systems (1TE722, 5 hp)

## Background
Truck platooning is a technology that enables two or more trucks to travel together in a coordinated way, using wireless communication and automated driving systems. The main benefit of this technology is to reduce fuel consumption and emissions, as well as to improve traffic flow and safety. The trucks form a close-following convoy, where the leading truck sets the speed and direction, and the following trucks automatically adjust their movements accordingly. The drivers remain in control of their vehicles at all times, and can choose to join or leave the platoon as they wish. Truck platooning is currently being tested and developed in various countries, with the aim of making it a viable option for long-distance freight transport in the future.

Some research has already been made in regards to platooning. In a [brief paper](https://www.sciencedirect.com/science/article/abs/pii/S0005109817301838) written by Ivo Herman, Steffi Knorn and Anders Ahlén a bidirectional system between the vehicles is looked at,. where a vehicle is not only reacting to what the vehicle in front is doing but also vice versa. This approach has given inspiration to this project.

## Objectives

The objective is to autonomously drive vehicles in a platoon in an energy efficient way. Reducing the distance between vehicles in the platoon will result in less drag for following vehicles, and thus less energy being lost. However, the tradeoff is that reducing the distance between the vehicles too much can be unsafe in the case of disturbances, such as up or down slopes, traffic, etc.

This project will be carried out using a set of robotic cars that can be controlled with microcontrollers. The microcontrollers will be programmed to utilize sensors such as an ultrasonic sensor for distance measurement, and a light sensor to detect a line that will be acting as a rail for the vehicles to follow.

## Setup

### Dependencies
- rpi.gpio
- rospy


Get ROS in a docker container to run on a computer:

```zsh
docker pull ros
docker pull osrf/ros:noetic-desktop-full
```

Then run the container with the following command:

```zsh
docker run --privileged --mount type=bind,source=$(pwd),destination=/workspace -it osrf/ros:noetic-desktop-full
```

We have also prepared a set of raspberry Pis that will act as the nodes places on vehicles.
Connect to the Pis (ROS nodes) through ssh.

### Laff-v1
```zsh
ssh laff@192.168.212.135
```

### Laff-v2
```zsh
ssh laff@192.168.212.214
```
