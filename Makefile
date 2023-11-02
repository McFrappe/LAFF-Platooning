SHELL := /bin/bash

install:
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
	sudo apt update
	sudo apt install ros-noetic-ros-base
	sudo apt install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo rosdep init
	rosdep update

	git clone https://github.com/naoki-mizuno/ds4drv --branch devel ~/ds4drv
	cd ~/ds4drv && sudo python3 setup.py install && sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger

	cd ~/laff-platooning && git submodule update --init --recursive
	cd ~/laff-platooning && sudo pip install -r requirements.txt
	source /opt/ros/noetic/setup.bash

cat:
	rm -rf build
	catkin_make

run_joyit:
	source devel/setup.bash
	roslaunch joyit vehicle.launch

run_joyit_pi:
	echo "source ./devel/setup.bash; roslaunch joyit vehicle.launch --screen" | sudo su

run_rcv:
	source devel/setup.bash
	roslaunch rcv vehicle.launch

run_rcv_pi:
	echo "source ./devel/setup.bash; roslaunch rcv vehicle.launch --screen" | sudo su