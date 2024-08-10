### Prerequisites

ROS 2 : Humble 

Gazebo : Fortress

### Demo

```console
$ colcon build --symlink-install
$ source install/setup.bash
$ ros2 launch ur_vs ur10_gz.launch.py 
$ ros2 launch ur_vs ur10_moveit2_demo.launch.py 
$ ros2 run ur_vs test_pose_goal.py
```

