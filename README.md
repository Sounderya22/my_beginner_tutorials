## ROS 2 Programming Assignment 3 - tf2, unit testing, bag files

This is the ROS package beginner_tutorials made for the ROS 2 Programming Assignment 3. 

### Build instructions

```bash
git clone https://github.com/Sounderya22/my_beginner_tutorials.git
cd my_beginner_tutorials/
colcon_build
source . install/setup.bash
```

### Running the publisher and subscriber node 

```bash
cd my_beginner_tutorials/
source . install/setup.bash
ros2 run beginner_tutorials server
```
In another terminal

```bash
cd my_beginner_tutorials/
source . install/setup.bash
ros2 run beginner_tutorials client
```

### Running through the launch file

```bash
cd ros2_ws/
source . install/setup.bash
ros2 launch beginner_tutorials service.launch.py publish_frequency:=2.0
```

### Dependencies

- Ubuntu 22.04 (If running locally)
- ROS2 Humble
- Git
- C++17


