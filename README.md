# Disc Golf Inventory Automation: RasPi

Raspberry Pi serves as the communication hub using a ROS framework for real-time data exchange and control of the microcontrollers.

## How to Run
Launching the main file to run individual modules:
* Go into the catkin_ws to find the ROS files: `cd catkin_ws`
* Go into SRC: `cd src`
* Go to the main file from GitHub: `cd raspi-main`
* Now, launch ROS: `roslaunch ros-raspi raspi_main.launch`

Additional Instructions:
* Making a file executable if need be: `chmod +x <filename>`
* Starting rosserial: `roslaunch ros-raspi rosserial_3.launch`
* If you want to run a specific node (only if you have not used roslaunch): `roscore`
* Starting a specific node: `rosrun ros-raspi <node_name>`

Nodes:
* `raspi_main.py` (runs all the hals)
* `raspi_ui.py` (runs the UI only)
* `raspi_hal__main_conveyor.py`
* `raspi_hal__intake.py`
* `raspi_hal__scale.py`
