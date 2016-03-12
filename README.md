## IIWA STACK
ROS indigo metapackage that contains ROS packages to work with the KUKA LBR IIWA R800/R820 (7/14 Kg).
___
### Features
- rosjava node running on the robot as a Sunrise RoboticApplication: supports ROS parameters, topics, services, actions
- SmartServo integration
  - default: position control (joint or cartesian)
  - joint or cartesian impedance control reconfigurable through a ROS service
  - Sunrise tool can be attached to the flange in order to consider its mass for impedance control
- monitor mode with gravity compensation, enabled through SmartPad keys
- optional autonomous publication of joint_states topic
- NTP synchronization if server is running on ROS master
- full MoveIt! integration
- Gazebo support

There are two RoboticApplications included in our software stack: 
- ROSMonitor does not perform any motion, but it constantly publishes the current 
state of the robot. This is suitable for procedures in the native teaching mode 
or in the custom-made gravity compensation mode.
- ROSSmartServo allows to use the Sunrise SmartServo control modes to move the robot. 
The motion is fully reconfigurable at runtime, allowing to switch from any control 
strategy to any other, and to change the respective parameters at runtime through a 
ROS service.

Both applications look for ROS parameters at startup, including the ID of the Sunrise 
tool to attach to the flange. The IP of the robot is automatically discovered, while 
the ROS master IP should be set in the configuration file included in the project.

The communication with the robot is implemented through ROS topics: a rosjava node 
runs in the context of a Sunrise application on the robot cabinet, which in turn 
performs a SmartServo motion. This approach allows great flexibility, as we can 
use a ROS service to reconfigure the ServoMotion at runtime (e.g. stiffness of 
certain joints or around cartesian axes). Even if not real-time, we never incurred 
in any communication problems on our setup: the TCP topics could scale up to 4 KHz 
without compromising the ordering of the received packets. However, we also plan 
to offer FRI as a drop-in replacement for ROS-topic communication as soon as it is stable. 

The state of the robot is published through a set of topics, and optionally through 
the joint_states topic for compatibility with the robot_state_publisher even when 
not using MoveIt!. The timestamps are correct if an NTP server runs on the ROS master, 
such that the system can also be used for latency-sensitive application.

The impedance control can be configured with respect to the stiffness and damping 
for each joint, or for each and about each direction in the cartesian space. It is 
possible to switch the control mode at runtime safely.
___
### Usage
__The features and usage of the stack are described in depth on its  [WIKI][8].__  
We **_strongly_** suggest to have a look at the wiki to have a better understanding of the code, both for its use and its extension.

___
### Acknowledgements
This repository takes inspiration from the work of :
- _Centro E. Piaggio_ and their [ROS interface for the KUKA LBR 4+][1]
- _Mohammad Khansari_ and his [IIWA-ROS communication inteface] [2] 
- _Robert Krug_ and his [IIWA URDF and Gazebo package][7]      

Most of the original files were completely refactored though.
___
### Overview
This packages contained in this repository are :
- __iiwa__ : the ROS metapackage
- __iiwa_control__: contains the joint and trajectory controllers used by MoveIt! and Gazebo.
- __iiwa_description__: URDF for both KUKA LBR IIWA R800 and R820.
- __iiwa_gazebo__: launch files to run a Gazebo simulation.
- __iiwa_hw__: implements the ROS hardware interface and the communication interface with the real robot (using iiwa_ros).
- __iiwa_moveit__: a MoveIt! configuration for controlling the robot (either a Gazebo simulation or a real one).
- __iiwa_msgs__: creates ROS messages to be used for communication with a real robot. 
- __iiwa_ros__: an interface to send and receive messages defined in iiwa_msgs to and from a real robot.
- __iiwa_ros_java__: the ROSJava interface to use on SunriseApplications - it allows to send and receive messages defined in iiwa_msgs.

### Contacts
Salvatore Virga : salvo.virga@tum.de     
Marco Esposito : marco.esposito@tum.de

[1]: https://github.com/CentroEPiaggio/kuka-lwr
[2]: https://bitbucket.org/khansari/iiwa.git
[3]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/msg/IIWAMsg.msg
[4]: https://bitbucket.org/khansari/iiwa/wiki/Home
[5]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/JavaNode/?at=master
[6]: http://git.lcsr.jhu.edu/cgrauma1/kuka_iiwa_shared
[7]: https://github.com/rtkg/lbr_iiwa
[8]: https://github.com/SalvoVirga/iiwa_stack/wiki
