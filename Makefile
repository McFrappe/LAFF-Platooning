setup:
	sudo apt install python3-pip
	sudo pip install -r requirements

cat:
	rm -rf build
	catkin_make

run:
	source devel/setup.bash
	roslaunch joyit vehicle.launch

run_pi:
	sudo su
	source devel/setup.bash
	roslaunch joyit vehicle.launch