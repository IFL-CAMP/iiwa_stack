## IIWA STACK
ROS indigo metapackage that contains ROS packages to work with the KUKA LBR IIWA.

### Acknowledgements
This repository takes inspiration from the work of :
- _Centro E. Piaggio_ and their [ROS interface for the KUKA LBR 4+][1]
- _Mohammad Khansari_ and his [IIWA-ROS communication inteface] [2]

### Overview
This packages contained in this repository are :
- __lbr_iiwa_control__: contains the controllers used by MoveIt!
- __lbr_iiwa_description__: URDF description of the KUKA LBR IIWA
- __lbr_iiwa_gazebo__: launch files to run a Gazebo simulation
- __lbr_iiwa_hw__: creates a bridge between the ROS hardware interfaces and the communication interfaces with the real robot (lbr_iiwa_ros).
- __lbr_iiwa_launch__: all the launch files to use the selected controller
- __lbr_iiwa_moveit__: a MoveIt! configuration for controlling the robot (either a Gazebo simulation or a real one)
- __lbr_iiwa_msg__: creates a message of type IIWAMsg as defined by [Khansari][3], to be used for communication with a real robot. 
- __lbr_iiwa_ros__: an interface to send and receive messages of type IIWAMsg to and from a real robot.

### Dependencies

The dependencies of each package in a catkin workspace can be installed by typing 
into a shell in the workspace directory:  
`rosdep install --from-paths src --ignore-src -r -y`

### Usage
You can either use a simulated robot with Gazebo or a real robot connected to the machine that runs ROS via its KONI Ethernet port.

For the real robot you have to set up the cabinet as described in [Khansari's wiki page][4], and you need to have a robotic application
that connects to your roscore and communicates over the topics _/iiwa/state_ and _/iiwa/command_ using messages of the type IIWAMsg. 
More on [lbr_iiwa_ros README](lbr_iiwa_ros/README.md)

You can either use the [Java code distributed by Khansari][5],
or our [IIWA Shared Library][6] available on JHU's git. (You can contact c.m.graumann@jhu.edu to have access to that).

### MoveIt!

#### Demo
This launch file loads a very simple fake controller that never fails (useful for evaulation of the inverse kinematics):  

`roslaunch lbr_iiwa_moveit demo.launch`
#### Gazebo Simulation
This launch configuration starts a Gazebo simulation that is controlled by MoveIt!: 

`roslaunch lbr_iiwa_moveit moveit_planning_execution.launch`
#### Real Robot
This is how MoveIt! can be connected to a real robot:

`roslaunch lbr_iiwa_moveit moveit_planning_execution.launch sim:=false`

[1]: https://github.com/CentroEPiaggio/kuka-lwr
[2]: https://bitbucket.org/khansari/iiwa.git
[3]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/msg/IIWAMsg.msg
[4]: https://bitbucket.org/khansari/iiwa/wiki/Home
[5]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/JavaNode/?at=master
[6]: http://git.lcsr.jhu.edu/cgrauma1/kuka_iiwa_shared