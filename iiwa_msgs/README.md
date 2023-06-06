# Message Generation

# CPP, Python, ...
These messages should be automatically generated when calling `catkin_make` or `catkin build` in the workspace.

## Java Messages - Docker
Since genjava has only support for Kinetic and earlier versions of ROS, a DOCKERFILE has been written.
Specifically, we use a fork of the genjava package which will generate the messages for Java 6, which is the latest Sunrise supports.
This file can be used to setup the build environment on any system that supports Docker.
To build the DOCKERFILE simply **navigate to this directory** in a shell and call:
```bash
docker build --tag iiwa_msgs:latest . 
```

Now, the messages can be generated and copied to the host.
```bash
docker run --name iiwa_msgs -it iiwa_msgs:latest /bin/bash -c \
  "source devel/setup.bash && \
  genjava_message_artifacts --verbose -p iiwa_msgs actionlib_msgs geometry_msgs std_msgs"
docker cp iiwa_msgs:/catkin_ws/devel/share/maven/org/ros/rosjava_messages rosjava_messages
docker rm iiwa_msgs
```

The generated files will end up in a directory called `rosjava_messages` in your current directory.
In the leaf directories you will find the .jar files which can be copied to `iiwa_ros_java/ROSJavaLib`.
Make sure to replace all four files since they depend on each other.
In the Sunrise Workbench, refresh the files view and edit the build path: remove the old versions and add the new versions from ROSJavaLib.