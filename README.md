## ROS 2 Programming Assignment 1 - Publisher/Subscriber

This is the ROS package beginner_tutorials made for the ROS 2 Programming Assignment 1. 

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
ros2 run beginner_tutorials talker
```
In another terminal

```bash
cd my_beginner_tutorials/
source . install/setup.bash
ros2 run beginner_tutorials listener
```

### Dependencies

- Ubuntu 22.04 (If running locally)
- ROS2 Humble
- Git
- C++17


