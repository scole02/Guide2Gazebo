# Guide to the Gazebo (Ignition) Simulator
This guide includes some code and helpful instructions for all things Gazebo (Ignition) including:
1. **Setting up Gazebo**
2. **Docker**
3. **Features and Community Examples**
4. **ROS2 Integration**
5. **SDF and URDF**
6. **SDF and Plugins**
7. **Topics**
8. **CMake**

---
<br/>

## 1. Setting up Gazebo
### Notes on Versioning
#### Gazebo Classic != Ignition Gazebo
[Gazebo (Classic)](http://classic.gazebosim.org/) is the predecessor of [Ignition Gazebo](https://gazebosim.org/home). The Ignition project started 7 years ago and functions similar to Classic, but is architecturally different. For most intents and purposes, code or tutorials for Gazebo Classic and Ignition will be completely different and bare little resemblance to each other.

#### Gazebo Classic --> Ignition Gazebo --> Gazebo

As of 8/2022 Ignition Gazebo is now changing back to just Gazebo. This is totally not confusing it's all in your head. 

#### Namespace Issues
The last planned "Ignition" release will be "Ignition Garden", after that a new naming scheme is planned. The Garden release uses a set of libraries shown here:
Library|Version
:-----:|:-----:
gz-cmake|3.x
gz-common|5.x
gz-fuel-tools|8.x
gz-sim|7.x
gz-gui|7.x
gz-launch|6.x
gz-math|7.x
gz-msgs|9.x
gz-physics|6.x
gz-plugin|2.x
gz-rendering|7.x
gz-sensors|7.x
gz-tools|2.x

Previous versions of Ignition, such as Fortress and Citadel, rely on older versions of the same packages **BUT** the `gz-` prefix is replaced with `ign-`. This may lead to some headache using **cmake** and finding packages, but more importantly, any C++ code that was made for previous releases (pre-Garden) of gazebo will not compile. They will all include header files with the old `ign-` prefix. I have not tested if simply changing the prefix to `gz-` will fix this problem, but this is something to be aware of.


### Installation

The following **Ignition Garden Binary** install commands work as of 8/2022. If they don't, the official installation guide is [here](https://gazebosim.org/docs/latest/install).

```bash
# Add Ignition's latest packages
sudo /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    sudo /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list' && \
    sudo /bin/sh -c 'wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -'

# Install the latest Ignition binaries
sudo apt-get -qq update && sudo apt-get install \
  ignition-garden
  ```
 
 ###  Testing installation
 Run the CLI using:  
 
 `gz    # older versions of ignition will use "ign" command`
 
The following output will represent which gazebo `.yaml` configuration files are visible to your system. Ideally you should see this:

```
user@ubuntu_machine:~$ gz
The 'gz' command provides a command line interface to the Gazebo Tools.

  gz <command> [options]

List of available commands:

  help:          Print this help text.
  sdf:           Utilities for SDF files.
  plugin:        Print information about plugins.
  launch:        Run and manage executables and plugins.
  model:         Print information about models.
  msg:           Print information about messages.
  sim:           Run and manage the Gazebo Simulator.
  gazebo:        Deprecated. Alias for sim.
  topic:         Print information about topics.
  service:       Print information about services.
  fuel:          Manage simulation resources.
  gui:           Launch graphical interfaces.
  log:           Record or playback topics.

Options:

  --force-version <VERSION>  Use a specific library version.
  --versions                 Show the available versions.
  --commands                 Show the available commands.
Use 'gz help <command>' to print help for a command.
```
### Troubleshooting
If you see less available commands then above, either you did not install all the libraries (each one has a corresponding library, see Namespace Issues above) or your system cannot find configuration files. 

If your system can not find any available configuration files you should see this:
```bash
user@ubuntu_machine:~$ gz
I cannot find any available 'gz' command:
	* Did you install any Gazebo library?
	* Did you set the GZ_CONFIG_PATH environment variable?
	    E.g.: export GZ_CONFIG_PATH=$HOME/local/share/gz
```

Setting the environment variable `GZ_CONFIG_PATH` to the directory containing the yaml config files will fix this. On my machine with **Ignition Garden Binaries** the directory with these config files looks like so:

```bash
user@ubuntu_machine:~$ ls /usr/share/gz
fuel8.yaml   gui7.yaml  gz-common5      gz-gui7     gz-math7  gz-plugin2     gz-sensors7  gz-transport12  gz1.completion.d  model7.yaml  plugin2.yaml     sim7.yaml         transportlog12.yaml
fuel_tools8  gz-cmake3  gz-fuel_tools8  gz-launch6  gz-msgs9  gz-rendering7  gz-sim7      gz-utils2       launch6.yaml      msgs9.yaml   sdformat13.yaml  transport12.yaml
```

If your machine has these config files in the same location, set the config variable appropiately:

```bash
export GZ_CONFIG_PATH=/usr/share/gz
```

---
<br/>

## 2. Docker
<img src="https://github.com/scole02/Guide2Gazebo/blob/main/doc/image/docker.png"></img>  
If you want to try out gazebo but don't want to clutter your current OS, the `gz-sim` github repo contains a whole section with Dockerfile(s) and instructions on how to build them [here](https://github.com/gazebosim/gz-sim/tree/gz-sim7/docker). 

### Troubleshooting
Using the `gz-sim:nightly` image, you may run into problems when attempting to open the gazebo GUI. In my case these issues were related to a **QT5 X server** plugin and the docker container unable to find my graphics card. Here were some of the errors I got when running the docker command `docker run --gpus gz-sim:nightly gz sim`:
```
QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-root'
libGL error: MESA-LOADER: failed to retrieve device information
intel_do_flush_locked failed: Input/output error
...
```
```
qt.qpa.xcb: could not connect to display 
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.
...
```

Run this command to set allow all clients to connect to xserver
`xhost +`

The following docker command should alleviate these problems:  
`docker run -it --net=host -e DISPLAY=$DISPLAY --device /dev/dri/card0:/dev/dri/card0 -v /tmp/.X11-unix/:/tmp/.X11-unix gz-sim:nightly bash`



## 3. Features and Community Examples 
### What the heck is Gazebo?
Gazebo is a full physics simulator that allows for the importing of entire worlds and models expressed using the [SDF format](http://sdformat.org/). Gazebo also has a very powerful plugin system that allows for precise monitoring and control of a world and the GUI presented to the user, all using the Gazebo C++ API. 

Here are some projects showing what is possible using SDF models, Gazebo plugins, and common robotics frameworks.

### Pure Gazebo ROV ([MBARI LRAUV](https://github.com/osrf/lrauv))

OSRF project that shows how custom UI, sensors, physics, communications, and data collection can be achieved using only Gazebo Plugins. 
![](https://global.discourse-cdn.com/standard10/uploads/gazebo/optimized/1X/973b1dbc28d0547136781b923748a6e780cf6c1e_2_690x379.jpeg)

<br/>

### Gazebo, ROS2, and ArduPilot ROV ([BlueROV2](https://github.com/clydemcqueen/orca4))
Project integrating Gazebo for physics, ROS2 for control, and ArduPilot for user interfacing.
![](https://github.com/clydemcqueen/orca4/raw/main/images/gazebo.png)



<br/>

### Gazebo, ROS2, and Moveit2 ([Panda Arm](https://github.com/AndrejOrsula/ign_moveit2_examples))
This project showcases using Gazebo and the Moveit ROS2 package together.

![](https://user-images.githubusercontent.com/22929099/147374613-ad15aa1a-deaf-4dcd-92b0-1a53d0097467.gif)


---
<br/>

## 4. ROS2 Integration
### Gazebo Bridge ([ros_ign_bridge](https://github.com/gazebosim/ros_gz/tree/galactic_to_ros2/ros_ign_bridge))
This package provides a bridge communcation layer that allows for the bidirectional exchange of messages between ROS2 and Gazebo. 

The table below shows which message types are supported by the bridge as of 8/2022. The `ignition::` namespace may have to be changed to `gz::` depending on your environment and install.
|            ROS type            | Ignition Transport type          |
|:------------------------------:|----------------------------------|
| std_msgs/Bool                  | ignition::msgs::Boolean          |
| std_msgs/ColorRGBA             | ignition::msgs::Color            |
| std_msgs/Empty                 | ignition::msgs::Empty            |
| std_msgs/Int32                 | ignition::msgs::Int32            |
| std_msgs/Float32               | ignition::msgs::Float            |
| std_msgs/Float64               | ignition::msgs::Double           |
| std_msgs/Header                | ignition::msgs::Header           |
| std_msgs/String                | ignition::msgs::StringMsg        |
| geometry_msgs/Quaternion       | ignition::msgs::Quaternion       |
| geometry_msgs/Vector3          | ignition::msgs::Vector3d         |
| geometry_msgs/Point            | ignition::msgs::Vector3d         |
| geometry_msgs/Pose             | ignition::msgs::Pose             |
| geometry_msgs/PoseArray        | ignition::msgs::Pose_V           |
| geometry_msgs/PoseStamped      | ignition::msgs::Pose             |
| geometry_msgs/Transform        | ignition::msgs::Pose             |
| geometry_msgs/TransformStamped | ignition::msgs::Pose             |
| geometry_msgs/Twist            | ignition::msgs::Twist            |
| mav_msgs/Actuators             | ignition::msgs::Actuators        |
| nav_msgs/OccupancyGrid         | ignition::msgs::OccupancyGrid    |
| nav_msgs/Odometry              | ignition::msgs::Odometry         |
| rosgraph_msgs/Clock            | ignition::msgs::Clock            |
| sensor_msgs/BatteryState       | ignition::msgs::BatteryState     |
| sensor_msgs/CameraInfo         | ignition::msgs::CameraInfo       |
| sensor_msgs/FluidPressure      | ignition::msgs::FluidPressure    |
| sensor_msgs/Imu                | ignition::msgs::IMU              |
| sensor_msgs/Image              | ignition::msgs::Image            |
| sensor_msgs/JointState         | ignition::msgs::Model            |
| sensor_msgs/LaserScan          | ignition::msgs::LaserScan        |
| sensor_msgs/MagneticField      | ignition::msgs::Magnetometer     |
| sensor_msgs/NavSatFix          | ignition::msgs::NavSat           |
| sensor_msgs/PointCloud2        | ignition::msgs::PointCloudPacked |
| tf_msgs/TFMessage              | ignition::msgs::Pose_V           |
| visualization_msgs/Marker      | ignition::msgs::Marker           |
| visualization_msgs/MarkerArray | ignition::msgs::Marker_V         |

<br/>  

### Gazebo Sim ([ros_ign_gazebo](https://github.com/gazebosim/ros_gz/tree/galactic_to_ros2/ros_ign_gazebo))

This package provides a node to spawn an entity into a Gazebo world, as well as ROS2 launch file examples that demonstrate how to start a Gazebo world.

<br/>

### Gazebo Images ([ros_ign_image](https://github.com/gazebosim/ros_gz/tree/galactic_to_ros2/ros_ign_image))

Gazebo has full support for simulating cameras and generating image data. This package allows for unidirectional communication of image data from Gazebo to ROS.

<br/>

### Gazebo Pointclouds ([ros_ign_point_cloud](https://github.com/gazebosim/ros_gz/tree/galactic_to_ros2/ros_ign_image))

Gazebo has full support for simulating depth cameras and generating pointcloud data. This package allows for unidirectional communication of pointcloud data from Gazebo to ROS.

<br/>

### ROS2 Control and Gazebo ([gz_ros2_control](https://github.com/ros-controls/gz_ros2_control))

This package allows models within gazebo to react to state updates of controllers implementing the `ros2_control` architecture. As of writing, this package **does NOT support Ignition Garden**.
![](https://github.com/ros-controls/gz_ros2_control/raw/master/img/ign_ros2_control.gif)

---
<br/>

## 5. SDF and URDF
[**SDFormat**](http://sdformat.org/) (Simulation Description Format), is an XML format that describes objects and environments for robot simulators, visualization, and control. **Gazebo** represents worlds and models (robots) using SDF.


[**URDF**](http://wiki.ros.org/urdf) (Universal Robot Description Format) is also an xml format that is used to describe robots, sensors, and scenes. This format is used within **ROS** and packages such as Moveit.

For more detail see these articles/tutorials:
* [ROS URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
* [Gazebo Classic: Make an SDF Model](https://classic.gazebosim.org/tutorials?tut=build_model&cat=build_robot#ComponentsofSDFModels) 
* [URDF vs. SDF – Link Pose, Joint Pose, Visual & Collision](https://automaticaddison.com/urdf-vs-sdf-link-pose-joint-pose-visual-collision/)


### Convert URDF to SDF
The CLI `gz sdf` provides a set of tools for converting and getting more detail about SDF files.

Before converting URDF to SDF, ensure that all [`xacro`](http://wiki.ros.org/xacro) has been parsed out of the source URDF. This can be done using the xacro CLI that is bundled with ROS2. Then use `gz sdf -p`:

```bash
source /opt/ros/$ROS_DISTRO/setup.sh  # find xacro command
xacro robot.urdf.xacro > robot.urdf   # parse out xacro tags
gz sdf -p robot.urdf > robot.sdf      # convert to sdf
```

This can also be done inside a ROS2 launch file using the package `ros_ign_gazebo`. In this example our xacro file takes two arguments and is located in the **installed** package, `robot_description` in the urdf directory, `/urdf/robot.xacro.urdf`. This launch file also spawns the urdf into the default gazebo world included with every install, `empty.sdf`.
```python
# ROS2 Launch file that converts xacro -> urdf, and spawns urdf into sdf world

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro
from os import path

def generate_launch_description(

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    arg1 = 1
    arg2 = 2
    
    description_path = os.path.join(get_package_share_directory("robot_description"))

    xacro_file = os.path.join(description_path, "urdf", "robot.urdf.xacro")

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc,  mappings={"arg1": arg1, "arg2": arg2})

    ignition_spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        output="screen",
        arguments=["-string", doc.toxml(), "-name", "robot_model", "-allow_renaming", "true"],
    )


    return LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_directory("ros_ign_gazebo"),
                            "launch",
                            "ign_gazebo.launch.py",
                        )
                    ]
                ),
                launch_arguments=[("ign_args", [" -v 5 empty.sdf"])], # -v N enables specified level of logging: error, debug, info, etc.
            ),
            ignition_spawn_entity,
            # Launch Arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )

```

### Troubleshooting
For checking general XML/SDF syntax errors use the command:
`gz sdf -k robot.urdf`

If the output SDF file produced has no body or is missing elements, make sure that **ALL links have inertia and mass**. This is done using an [inertial XML tag](http://sdformat.org/spec?ver=1.9&elem=link#link_inertial) that might look like this:

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="box">
    <pose>0 0 0.5 0 0 0</pose>

    <link name="link">
      <pose>0 0 0 0 0 0</pose>
        
         <inertial> <!-- 'gz sdf' needs this tag for every link -->
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
          <mass>1.0</mass>
        </inertial>

        <collision name="my_collision">
          ...
        </collision>

        <visual name="my_visual">
          ...
        </visual>

        <sensor type="camera" name="my_sensor">
          ...
        </sensor>
        
    </link>
  </model>
</sdf>
```

### URDF to Gazebo Model
To make your robot appear as a model that can be included in a gazebo world, create a directory with the following structure:

```
gazebo_models/
├─ robot/
│  ├─ meshes/
│  │  ├─ mesh.stl
│  ├─ model.config
│  ├─ model.sdf
|
├─ another_model/
│  ├─ meshes/
│  │  ├─ mesh.stl
│  ├─ model.config
│  ├─ model.sdf
...
```

Now in a gazebo world SDF file you can include your model:

```xml
<!-- path: <SOME_PATH>/gazebo_worlds/robot_world.sdf -->
<?xml version="1.0" ?>

<sdf version="1.6">
  <world name="robot_world">

    <include>
      <uri>model://robot</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
      
  </world>
</sdf>
```

Gazebo will be unable to find your model and world unless you specify where to look for simulation resources.
```bash
export GZ_SIM_RESOURCE_PATH=<SOME_PATH>/gazebo_models/:<SOME_PATH>/gazebo_worlds/
gz sim robot_world.sdf # start the world
```




---
<br/>

## 6. SDF and Plugins

For any complex logic or data processing beyond the default, a [Gazebo plugin](https://gazebosim.org/api/gazebo/2.10/createsystemplugins.html) will have to be used. These are very powerful components and can have access to the entirety of a world's parameters as well as any entities (models) that exist within it. 

Plugins can be used to:
* Apply special physics to a world globally
* Apply special physics to a model
* Add Sensors to a model 
* Add Actuators to a model
* Rendering custom graphics
* Control the camera
* Change the UI
* Communicate with external software

### Using Plugins (Physics/Sensors)

In general the first step to using a plugin is adding its library to the world file in which it is going to be used.

```xml
<!-- path: <SOME_PATH>/gazebo_worlds/robot_world.sdf -->
<?xml version="1.0" ?>

<sdf version="1.6">
  <world name="robot_world">
      
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
      
    <include>
      <uri>model://robot</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
      
  </world>
</sdf>
```

For plugins that are model independent, only adding them to the world file should suffice. When adding a plugin that is part of a model, such as an **Altimeter Sensor**, we must specify the *plugin system* in the world file **AND** the sensor itself in the model file.

```xml
<!-- in world sdf -->
<!-- plugin system is responsible for loading the sensor plugin  -->
    <plugin 
      filename="ignition-gazebo-altimeter-system"
      name="ignition::gazebo::systems::Altimeter">
    </plugin>

  </world>
</sdf>
```
```xml
<!-- in model sdf -->
      <sensor name="altimeter" type="altimeter">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
          <visualize>true</visualize>
          <topic>altimeter</topic>
          <enable_metrics>true</enable_metrics>
          <altimeter>
            <vertical_position>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.05</stddev>
              </noise>
            </vertical_position>
            <vertical_velocity>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.05</stddev>
              </noise>
            </vertical_velocity>
          </altimeter>
        </sensor>        
  </model>
</sdf>
```
---
## 7. Topics
Similiar to ROS, Gazebo has a topic system where messages are published and subscribed to as the world and model change over time. These topics are published and subscribed to by plugins.

### Topic CLI 
Here are some helpful commands for interacting with topics:

```bash
# list all topics currently being published
gz topic -l                                 
# "echo messages published to my_topic"
gz topic -e -t my_topic                     
# publish double msg to "my_topic" 
gz topic -t my_topic -m gz.msgs.Double -p 'data : 1.0'
```


## 8. CMake

Alot of cmake build problems can be fixed by setting your ignition version:

`user@ubuntu_machine:~$export IGNITION_VERSION=garden;`

If this doesn't fix the problem...   
<img src="https://github.com/scole02/Guide2Gazebo/blob/main/doc/image/cmake_pain.jpeg" width="200">

