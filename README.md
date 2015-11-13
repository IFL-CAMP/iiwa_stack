## IIWA STACK
ROS indigo metapackage that contains ROS packages to work with the KUKA LBR IIWA R800/R820 (7/14 Kg).
___
### Features
- obtain the current state of a KUKA IIWA LWR and command the robot using ROS Messages (either custom - like iiwa_msgs - or standard ones).
- make use of the functionalities of Moveit! with a KUKA IIWA LWR. 
- Gazebo simulation.

The communication with the robot is implemented through ROS topics: a rosjava node runs in the context of a Sunrise application on the robot cabinet, which in turn performs a SmartServo motion. This approach allows great flexibility; we also plan to offer a service to reconfigure the ServoMotion at runtime (e.g. stiffness of certain joints or around cartesian axes). Though not real-time, we never incurred in any communication problems on our setup.

We also plan to offer FRI as a communication option as soon as it is stable. 

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

[1]: https://github.com/CentroEPiaggio/kuka-lwr
[2]: https://bitbucket.org/khansari/iiwa.git
[3]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/msg/IIWAMsg.msg
[4]: https://bitbucket.org/khansari/iiwa/wiki/Home
[5]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/JavaNode/?at=master
[6]: http://git.lcsr.jhu.edu/cgrauma1/kuka_iiwa_shared
[7]: https://github.com/rtkg/lbr_iiwa
[8]: https://campgit.in.tum.de/ros/iiwa_stack/wikis/home
