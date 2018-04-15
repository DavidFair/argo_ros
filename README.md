# Ros Driver for the Argo

# Table of Contents
* 1\. [Requirements](#requirements)
* 2\. [Building ROS Node](#building-ros-node)
* 3\. [Running the ROS Node](#running-the-ros-node)
  * 3.1\. [Setting speeds](#setting-speeds)
  * 3.2\. [Getting telemetry](#getting-telemetry)
* 4\. [Running node unit tests](#running-node-unit-tests)

## Requirements

The following dependencies are required:

- ROS (Hydro or later). See [ROS Wiki](http://wiki.ros.org/ROS/Installation).
- cmake
- git

Everything except ROS can be installed with the following command

```sh
sudo apt-get install cmake git
```

## Building the ROS node

### (Optional) Creating a separate working folder for the node:
1. Create a folder to build in - for example `ros`
2. cd into the folder
3. Make a folder called src
4. cd into src

For example:

```sh
mkdir -p ros/src
cd ros/src
```

### Building

1. cd into the working ROS folder
2. cd into the src folder
3. Clone the repository
4. cd back to working ROS folder
5. Run `catkin_make` on the working folder
6. Execute `source devel/setup.bash` to add the build node to the path

For example:

```sh
cd ros/src
git clone https://github.com/DavidFair/argo_ros.git
cd ..
catkin_make
source devel/setup.bash
```

## Running the ROS node

1. cd into the working ROS folder
2. Ensure the path has been updated, if not run `source devel/setup.bash`
3. In a separate terminal start `roscore` (or a background thread)
4. Run the node with the Arduino connected: `rosrun argo_driver argo_driver_node`


For example:

```sh
cd ros
source devel/setup.bash
roscore &
rosrun argo_driver argo_driver_node
```

### Setting Speeds

ROS services are used to set speeds. See the [ROS Wiki for more information](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams).

A list of available services can be found in a separate terminal whilst the node is running with the following command:

`rosservice list`

A list of expected parameters and types can be found with the following command, where *<service_name>* is the name of the service:

`rosservice type <service_name> | rossrv show`

A service can be called as follows, where *<service_name>* is the name of the service, and *<parameter_list>* is the list of expected parameters:

`rosservice call <service_name> <parameter_list>`

**Note:** To send negative values the following syntax is required:

`rosservice call <service_name> -- <parameter_list>`

For example to set a speed first forward at 1 meter per second then negative 2 meters per second:

```sh
# Prints that a left wheel and right wheel param is expected
rosservice type /argo_driver/set_target_speed | rossrv show

rosservice call /argo_driver/set_target_speed 1 1
sleep(10) # Wait 10 seconds
rosservice call /argo_driver/set_target_speed -- -2 -2
```

### Getting telemetry

ROS topics are published which contains the telemetry from the driver and vehicle. See the [ROS Wiki for more information](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)

A list of topics published by the node can be found with the following command:

`rostopic list`

An individual topic can be echoed to the console as with the following command, where *<topic_name>* is the name of the desired topic:

`rostopic echo <topic_name>`

For example to see the current vehicle speed the following command is executed:

```sh
# Find the name of the topic
rostopic list

rostopic echo /argo_driver/current_speed
```

## Running node unit tests

Prerequisite: Build the ROS node: see [Building the ROS node](#building-the-ros-node).

1. Ensure `roscore` is running either in a separate terminal or in the background
2. (Optional) Build unit tests only with `catkin_make tests`
3. Build and run unit tests with `catkin_make run_tests_argo_driver`

For example:

```sh
cd ros
roscore &

# Build the tests first optionally
catkin_make tests

# Build and run tests together
catkin_make run_tests_argo_driver
```