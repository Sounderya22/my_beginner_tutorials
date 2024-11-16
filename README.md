## ROS 2 Programming Assignment 3 - tf2, unit testing, bag files

This is the ROS package beginner_tutorials made for the ROS 2 Programming Assignment 3. 

### Get the code

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Sounderya22/my_beginner_tutorials.git
```

### Build instructions

```bash
cd ros2_ws/
colcon_build
source . install/setup.bash
```

### Running the talker node 

```bash
cd my_beginner_tutorials/
source . install/setup.bash
ros2 run beginner_tutorials talker
```

### Inspecting TF frames

#### To verify the tf_frames, run
```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo talk world
```
This should output the transform information between talk and world

#### To get the output in a pdf file, run
```bash
ros2 run tf2_tools view_frames
```
This will listen to the tf2 transform data published by the system and generate a .pdf and .gv containing a graphical representation of the transform tree

### Running ROS test

```bash
colcon test  --return-code-on-test-failure --event-handlers console_cohesion+
```
This should output "All tests passed" in the terminal

### ROS Bag

#### To record rosbag with the launch file
Run the talker node
```bash
cd ros2_ws/
source install/setup.bash
ros2 run beginner_tutorials talker
```
In another terminal
```bash
cd ros2_ws/
source install/setup.bash
ros2 launch beginner_tutorials record_bag.launch.py
```

#### To inspect the bag file
Navigate to the results directory

```bash
cd ros2_ws/src/my_beginner_tutorials/results
ros2 bag info recording_2024-11-15_17-59-59
```
Replace the bag file name accordingly

#### To play the bag file with the listener node
 Run the listener node

```bash
cd ros2_ws/
source . install/setup.bash
ros2 run beginner_tutorials listener
```
In another terminal, play the bag file

```bash
cd ros2_ws/
source . install/setup.bash
cd src/my_beginner_tutorials/results
ros2 bag play recording_2024-11-15_17-59-59
```

### Dependencies

- Ubuntu 22.04 (If running locally)
- ROS2 Humble
- tf2_ros package
- Git
- C++17


