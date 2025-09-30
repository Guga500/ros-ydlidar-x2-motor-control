not 100% cant niavigation
ข้างในจะมี ไฟล์ my_robot ใช้ ros noetic หุ่นยนขนส่ง
         ไฟล์ tur ใช้ ros2 rolling เป็น turtle sim joystick
         วิธีใช้ tur สามารถอ่าใน readme ได้
how to install วิธีโหลดและใช้
1.download
ถ้าโหลดเป็นไฟล์ zip ให้แตกไฟล์แล้วลากไฟล์ my_robot เข้า workspace เลย
cd ~/catkin_ws/src
git clone https://github.com/Guga500/ros-ydlidar-x2-motor-control.git
cd ~/catkin_ws/src
mv ros-ydlidar-x2-motor-control/my_robot .
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
5.คำสั่ง launch จะอยู่ใน readmeRL
