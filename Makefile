SHELL := /bin/bash
BRANCH ?= main

PASSWORD ?= laff
IP := $(shell ip addr show wlan0 | grep -Po4 'inet \K[\d.]+')
VEHICLE_ID := $(shell cat /tmp/VEHICLE_ID || echo 'vehicle_0')
ROS_MASTER_URI := $(shell cat /tmp/ROS_MASTER_URI || echo 'http://localhost:11311')


install:
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt install -y curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
	sudo apt update
	sudo apt -y install ros-noetic-ros-base
	sudo apt -y install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential g++ libusb-1.0-0-dev libwiringpi-dev tightvncserver bluez
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo rosdep init
	rosdep update

	git clone https://github.com/naoki-mizuno/ds4drv --branch devel ~/ds4drv
	cd ~/ds4drv && sudo python3 setup.py install && sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/

	git config --global --add safe.directory /home/laff/laff-platooning
	cd ~/laff-platooning && git submodule update --init --recursive
	cd ~/laff-platooning && sudo pip3 install -r requirements.txt
	source /opt/ros/noetic/setup.bash

	sudo cp ~/laff-platooning/src/pixy2_ros/pixy2_node/pixy2/src/host/linux/pixy.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger

	sudo cp ~/laff-platooning/orchestrator/laff.service /lib/systemd/system
	sudo systemctl enable laff.service
	sudo systemctl start laff.service
	sudo loginctl enable-linger laff

	echo 'laff ALL=(ALL) NOPASSWD:ALL' | sudo EDITOR='tee -a' visudo

local-update:
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "date -s \"$(shell wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z\""
	git pull
	git submodule update --recursive --remote

update:
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "date -s \"$(shell wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-8)Z\""
	git config --global --add safe.directory /home/laff/laff-platooning
	cd /home/laff/laff-platooning
	git reset --hard
	git pull
	git checkout $(BRANCH)
	git pull
	git submodule update --recursive --remote
	rm -rf build
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "cd /home/laff/laff-platooning; source devel/setup.bash || source /opt/ros/noetic/setup.bash; catkin_make"

cat:
	sudo rm -rf build
	catkin_make

debug_listener:
	source devel/setup.bash
	ROS_IP=$(IP) ROS_MASTER_URI=$(ROS_MASTER_URI) rostopic echo /$(VEHICLE_ID)/debug

run_joyit:
	source devel/setup.bash
	ROS_IP=$(IP) roslaunch joyit vehicle.launch

run_joyit_pi:
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "cd /home/laff/laff-platooning; source devel/setup.bash; ROS_IP=$(IP) ROS_MASTER_URI=$(ROS_MASTER_URI) roslaunch joyit vehicle.launch --screen --pid /tmp/laff.pid vehicle_id:=$(VEHICLE_ID)"

run_rcv:
	source devel/setup.bash
	ROS_IP=$(IP) ROS_MASTER_URI=$(ROS_MASTER_URI) roslaunch rcv vehicle.launch

run_rcv_pi:
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "cd /home/laff/laff-platooning; source devel/setup.bash; ROS_IP=$(IP) ROS_MASTER_URI=$(ROS_MASTER_URI) roslaunch rcv vehicle.launch --screen --pid /tmp/laff.pid vehicle_id:=$(VEHICLE_ID)"

run_rcv_joystick:
	source devel/setup.bash
	ROS_IP=$(IP) ROS_MASTER_URI=$(ROS_MASTER_URI) roslaunch rcv vehicle_joystick.launch

run_rcv_joystick_pi:
	echo $(PASSWORD) | sudo -S sleep 1 && sudo su - root -c "cd /home/laff/laff-platooning; source devel/setup.bash; ROS_IP=$(IP) ROS_MASTER_URI=$(ROS_MASTER_URI) roslaunch rcv vehicle_joystick.launch --screen --pid /tmp/laff.pid vehicle_id:=$(VEHICLE_ID)"
