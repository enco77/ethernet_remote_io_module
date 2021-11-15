# ethernet_remote_io_module
ROS wrapper for modbus client of remote IO module
### Installation
<br>cd catkin_ws/src</br>
<br>git clone https://github.com/enco77/ethernet_remote_io_module.git</br>
<br>cd ..</br>
<br>rosdep install --from-paths src --ignore-src -r -y</br>
<br>catkin_make</br>
### Usage
<br>Specify your modbus server IP address in config file</br>
<br>Then run </br>
<br>roslaunch ethernet_remote_io_module remote_io.launch</br>
#### For ROS Melodic run commands below to use python3
<br>sudo apt-get install python3-pip python3-yaml</br>
<br>sudo pip3 install rospkg catkin_pkg</br>
