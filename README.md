## ROS 2 Programming Assignment 2 - Services, Logging and Launch file

This is the ROS package beginner_tutorials made for the ROS 2 Programming Assignment 1. 

### Build instructions

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Sounderya22/my_beginner_tutorials.git
cd ..
colcon_build
source . install/setup.bash
```

### Running the publisher and subscriber node 

```bash
cd ros2_ws/
source . install/setup.bash
ros2 run beginner_tutorials server
```
In another terminal

```bash
cd ros2_ws/
source . install/setup.bash
ros2 run beginner_tutorials client
```

### Running through the launch file

```bash
cd ros2_ws/
source . install/setup.bash
ros2 launch beginner_tutorials service.launch.py 
```

### Dependencies

- Ubuntu 22.04 (If running locally)
- ROS2 Humble
- Git
- C++17


