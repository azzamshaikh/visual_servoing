# Visual Servoing
### Azzam Shaikh

This repo contains an implementation of a visual servoing algorithm to move a 2 link (revolute) robot from an initial pose to a reference, goal pose, as shown below.

![alt text](./assets/init_and_ref_poses.png)

The algorithm controls the robot to move to the goal position via the trajectory shown in the image below.

![alt text](./assets/trajectory.png)

A full write up is available in the `docs` folder.

## Dependencies

This solution was built in Ubuntu 22.04 using ROS2 Humble and Gazebo 11.

## Building the Workspace

Extract the `src` folder to a clean directory. 

In the parent folder of the `src` directory, run the following command:

```
colcon build && source install/setup.bash
```

## Running the code

Run the following command in the terminal:

```
ros2 launch rrbot_gazebo visual_servoing.launch.py
```

