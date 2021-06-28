# pybullet_ros

A bridge between [ROS](https://www.ros.org/) and [PyBullet](https://pybullet.org/wordpress/) for simulating robots.

<img src="https://github.com/domire8/pybullet_ros/blob/development/common/images/r2d2_rviz.png" alt="drawing" width="600"/>

# Project status

This project is in a medium stage and presents with the following features:

- body velocity control: Subscription to cmd_vel topic and apply desired speed to the robot (without noise)
- joint control: Joint position, velocity and (effort) control for all joints on the robot
- sensors: Odometry, joint states (joint position, velocity and effort feedback), laser scanner, RGB camera

The main implementation is done [here](ros/src/pybullet_ros/pybullet_ros.py), using several helper
classes ([Simulation](ros/src/pybullet_ros/pybullet_sim.py)
, [RobotDescription](ros/src/pybullet_ros/pybullet_robot_description.py)
, [Robot](ros/src/pybullet_ros/pybullet_robot.py), and [FuncExecManager](ros/src/pybullet_ros/function_exec_manager.py))
.

# Usage with Docker

To run the simulation with Docker, build and run the image with

```bash
cd pybullet_ros/docker
bash build-run.sh
```

If using NVidia GPUs, make sure to set the flag `USE_NVIDIA_TOOLKIT` to true before running the script.

Developers can optionally use the `remote-dev.sh` script, which will set up a container with a shared volume to this
directory.

# Local installation

The following instructions have been tested under ubuntu 20.04
and [ROS noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).

This wrapper requires that you have PyBullet installed, you can do so by executing:

        sudo -H pip3 install pybullet

Additionally, clone this repository inside
your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace), compile (catkin build) and source your
devel workspace (as you would normally do with any ROS pkg).

In case you need to simulate RGBD sensor then install opencv for python3 and ros cv bridge:

        sudo -H pip3 install opencv-python
        sudo apt install ros-noetic-cv-bridge

# Test the simulator

Two robots for testing purposes are provided: Acrobat and R2D2. They can be found [here](common/test/urdf).

## R2D2

Start the simulation of a simple R2D2 robot by executing:

```bash
roslaunch pybullet_ros r2d2.launch
```

You should now be able to see the robot in the PyBullet GUI.

### Send position control commands to the robot

Publish a float message to the following topic:

```bash
rostopic pub /r2d2/position_controller/command std_msgs/Float64MultiArray  "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
 [0., 0., 0., 0., 1.]" --once
```

This should rotate the last joint (the "neck" should turn).

### Position command GUI

Move in position control with a convenient GUI:

```bash
roslaunch pybullet_ros position_cmd_gui.launch robot_name:=r2d2
```

A GUI will pop up, use the slides to control the joints in position control.

NOTE: This GUI should not be active while sending velocity or effort commands!

### Send joint velocity or effort control commands to the robot

Before sending commands, make sure that the position control GUI is not running!

Publish a float msg to the following topics:

Velocity control:

```bash
rostopic pub /r2d2/velocity_controller/command std_msgs/Float64MultiArray  "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
 [0., 0., 0., 0., 2.]" --once
```

Effort control:

```bash
rostopic pub /r2d2/effort_controller/command std_msgs/Float64MultiArray  "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
 [0., 0., 0., 0., -2000.]" --once
```

The robot should now move in velocity or effort control mode with the desired speed/torque.

## Acrobat

To launch the Acrobat robot, do

```bash
roslaunch pybullet_ros acrobat.launch
```

This will launch the simulation and automatically bring up the position command GUI too.

## Visualize tf data and robot model in RViz

By default, RViz is launched alongside with the `.rviz` configuration corresponding to the current robot. It's used to
visualize the TF tree and the robot model. It can be disabled when launching the simulation with the `rviz_bringup`
argument set to `false`.

## Franka Panda

Additionally, if using the Docker containers, or if
the [franka_robot_description](https://github.com/domire8/franka_panda_description.git) package is cloned in the local
ROS workspace, the Franka Panda robot can be used too:

```bash
roslaunch pybullet_ros franka.launch
```

## Services offered by the simulator

Reset simulation, of type std_srvs/Trigger, which means it takes no arguments as input, it calls
pybullet.resetSimulation() method (TODO).

```bash
rosservice call /pybullet_ros/reset_simulation "{}"
```

Pause or unpause physics, prevents the wrapper to call stepSimulation():

```bash
rosservice call /pybullet_ros/pause_physics "{}"
rosservice call /pybullet_ros/unpause_physics "{}"
```

# Launch arguments and config file

The following parameters can be used to customize the behavior of the simulator
in [this](ros/launch/pybullet_ros.launch) launch file:

- `config_file`: A yaml file specifying the [plugins](#pybullet-ros-plugins) that should be included in the simulation
  as well as other parameters like a list of all robots present in the simulation, the simulation loop rate, the gravity
  and other necessary parameters for certain plugins. See [here](ros/config/pybullet_params_example.yaml) for an
  extensive example.

- `plugin_import_prefix`: Allow environment plugins located in external packages to be recognized by PyBullet ROS.

- `environment`: The name of the python file (has to be placed inside `plugins` folder) without the .py extension that
  implements the necessary custom functions to load an environment via python code, e.g using functions like
  self.pb.loadURDF(...) See "environment" section below for more details.

- `pybullet_gui`: Whether you want to visualize the simulation in a GUI or not.

- `gui_options`: Expose GUI options to the user, for example to be able to maximize screen
  -> `gui_options="--width=2560 --height=1440"`.

-`rviz_bringup`: Whether RViz should be launched or not.

-`rviz_config`: RViz config file.

-`start_paused`: Whether the simulation should be launched paused or not.

-`parallel_plugin_execution`: Whether the plugins should run in parallel or not.

-`use_deformable_world`: Set this parameter to true in case you require soft body simulation.

However, the `pybullet_ros.launch` file should not be used directly, but it should be embedded in a top level launch
file that launches additional robot-specific stuff. More precisely, for each robot, the launch file should contain the
following `group`:

```xml

<group ns="r2d2">
    <arg name="urdf_path" default="$(find pybullet_ros)/common/test/urdf/r2d2_robot/r2d2.urdf.xacro"/>
    <param name="urdf_path" value="$(arg urdf_path)"/>
    <param name="pose_x" value="0.0"/>
    <param name="pose_y" value="0.0"/>
    <param name="pose_z" value="0.7"/>
    <param name="pose_yaw" value="0.0"/>
    <param name="fixed_base" value="False"/>
    <param name="use_inertia_from_file" value="True"/>

    <!-- upload urdf model to ROS param server -->
    <param name="robot_description" command="$(find xacro)/xacro $(arg urdf_path)"/>

    <!-- robot_state_publisher, publish tf based on /joint_states topic and robot_description param -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen"/>
</group>
```

where more parameters can be specified. It is important that the namespace of the `group` (here "r2d2") corresponds with
the names specified in the config file! Refer to [this directory](ros/launch/robots) for examples.

- `urdf_path`: The path to the urdf(.xacro) file to load the robot from
- `pose_`: The values of the pose in which the robot should be spawned in the world.
- `fixed_base`: Whether to fix the first link of the robot to the world, useful for fixed base robots.
- `use_inertia_from_file`: If true, PyBullet will compute the inertia tensor based on mass and volume of the collision
  shape.

# PyBullet ROS Plugins

What is a PyBullet ROS plugin?

At the core of PyBullet ROS, there is the following workflow:

<img src="https://github.com/domire8/pybullet_ros/blob/development/common/images/main_loop.png" alt="drawing" width="200"/>

Basically, the code iterates over all registered plugins, runs their `execute` function, and after doing it for all
plugins, the simulation is stepped one time step.

## Plugin creation

This section shows you how you can create your own plugin, to extend this library with your own needs.

NOTE: Before creating a PyBullet ROS plugin, make sure your plugin does not exist already. Check available
plugins [here](https://github.com/domire8/pybullet_ros/blob/development/ros/src/pybullet_ros/plugins).

To ease the process, a templated is provided
[here](https://github.com/domire8/pybullet_ros/blob/development/ros/src/pybullet_ros/plugins/plugin_template.py).

Using th PyBullet [documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#)
you should be able to access all the functionality that the PyBullet API provides.

## Environment plugin (TODO)

To load an environment (URDF, SDF, etc) we provide with a particular one time plugin under "plugins/environment.py".

This is loaded during runtime via importlib based upon the value of the ```~environment``` parameter.

The recommended way is to set the "environment" parameter to point to a python file which has to be placed under "
plugins" folder (just as any other plugin).

Then set the environment parameter to be a string with the name of your python file, e.g. ```my_env.py``` but without
the .py, therefore only : ```my_env```.

Then inside my_env.py inherit from Environment class provided in plugins/environment.py and override
the ```load_environment_via_code``` method.

A template is provided under plugins/environment_template.py to ease the process.

As mentioned before, the code inside the method "load_environment_via_code" will be called one time only during
puybullet startup sequence.

## Wait a second... bullet is already integrated in gazebo. Why do I need this repository at all?

Well thats true, bullet is integrated in gazebo, they have much more plugins available and their api runs much faster as
it is implemented in C++.

I myself also use gazebo on a daily basis! , however probably some reasons why this repository be useful are because is
very easy and fast to configure a rapid prototype.

Your urdf model does not need to be extended with gazebo tags, installation is extremely easy from pypi and there are
lots of examples in pybullet available (
see [here](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/examples)). Additionally, its in
python! so is easier and faster to develop + the pybullet documentation is better.

## Topics you can use to interact with this node (TODO)

```/joint_states``` (sensor_msgs/JointState) this topic is published at the ```pybullet_ros/loop_rate```
parameter frequency (see parameters section for more detail). This topic gets listened by the robot state publisher
which in turn publishes tf data to the ROS ecosystem.

```/tf``` - This wrapper broadcats all robot transformations to tf, using the robot state publisher and custom plugins.

```/scan```- Using the lidar plugin you get laser scanner readings of type sensor_msgs/LaserScan.

```/odom``` - Using the odometry plugin, robot body odometry gets published (nav_msgs/Odometry).

```/cmd_vel``` - Using the body_vel_control plugin, the robot will subscribe to cmd_vel and exert the desired velocity
to the robot.

```<joint_name>_<xtype>_controller/command``` - replace "joint_name" with actual joint name and "xtype"
with [position, velocity, effort] - Using the control plugin, you can publish a joint command on this topic and the
robot will forward the instruction to the robot joint.

```/rgb_image``` - The camera image of type (sensor_msgs/Image)

# Authors / Maintainers

For any questions or further explanations, please contact the authors.

- Dominic Reber (dominic.reber@epfl.ch)

