# Setup

Download interbotix_xsarm_movit folders/files from interbotix_ros_manipulators. Compile with `catkin_make` and set the source.

# Running

In one terminal window:
```
roslaunch interbotix_xsarm_moveit xsarm_moveit.launch robot_model:=vx300s use_actual:=true dof:=6
```

In another terminal window:
```
python3 mover.py
```
To rotate the gripper:
```
eve.rotate_gripper(pi/2)
```

To take a Cartesian step:
```
eve.step([[0, 1, 0], [0, -2, 0], [0, 2, 0]])
```
This is currently not working that well.