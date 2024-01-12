# Setup

Download interbotix_xsarm_movit folders/files from interbotix_ros_manipulators. Compile with `catkin_make` and set the source.

# Running

In one terminal window:
```
roslaunch interbotix_xsarm_moveit xsarm_moveit.launch robot_model:=vx300s use_actual:=true dof:=6 use_sim_time:=true
```

In another terminal window:
```
python3 mover.py
```
