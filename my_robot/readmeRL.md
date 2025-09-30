not 100% cant niavigation
how to install วิธีโหลดและใช้
1.download
cd ~/catkin_ws/src
git clone https://github.com/Guga500/ros-ydlidar-x2-motor-control.git
2.install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
3.comp
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
4.test
rospack list | grep my_robot
5.คำสั่ง launch

1 ROS Serial Arduino
rosrun rosserial_python serial_node.py /dev/ttyUSB1
2 SLAM
roslaunch my_robot slam.launch
3.Teleop Joystick
roslaunch my_robot joystick_teleop_advanced.launch 
4 Navigation 
roslaunch my_robot navigation.launch(not 100%)

