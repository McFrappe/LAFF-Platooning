SHELL := /bin/bash
BRANCH ?= main

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
	cd ~/ds4drv && python3 setup.py install && sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules
	sudo udevadm trigger

	git clone https://github.com/mcfrappe/laff-platooning ~/laff-platooning
	cd ~/laff-platooning && git submodule update --init --recursive
	cd ~/laff-platooning && sudo pip install -r requirements.txt

	sudo cp ~/laff-platooning/orchestrator/laff.service /lib/systemd/system
	sudo systemctl enable laff.service
	sudo systemctl start laff.service
	sudo loginctl enable-linger laff


update:
	git reset --hard
	git checkout $(BRANCH)
	git pull
	source devel/setup.bash
	rm -rf build
	catkin_make

cat:
	rm -rf build
	catkin_make

run:
	source devel/setup.bash
	roslaunch joyit vehicle.launch

run_pi:
	echo "source devel/setup.bash; roslaunch joyit vehicle.launch --screen --pid /tmp/laff.pid" | sudo su
