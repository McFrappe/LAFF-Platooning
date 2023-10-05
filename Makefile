install:
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
	sudo apt update
	sudo apt install ros-noetic-ros-base
	sudo apt install python3-pip python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
	sudo pip install -r requirements.txt
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	sudo rosdep init
	rosdep update

setup:
	sudo apt install python3-pip
	sudo pip install -r requirements.txt

cat:
	rm -rf build
	catkin_make

run: cat
	. devel/setup.bash
	roslaunch joyit vehicle.launch

run_pi: cat
	sudo su
	. devel/setup.bash
	roslaunch joyit vehicle.launch --screen