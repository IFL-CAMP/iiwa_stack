## IIWA STACK
ROS Indigo/Kinetic metapackage for the KUKA LBR IIWA R800/R820 (7/14 Kg).

**Current version : v-1.3.0 for Sunrise 1.10 - 1.16**    
[Using a previous version of Sunrise?](https://github.com/SalvoVirga/iiwa_stack/wiki/FAQ#which-version-of-sunriseossunrise-workbench-is-supported)    

[![Build Status](https://travis-ci.org/IFL-CAMP/iiwa_stack.svg?branch=master)](https://travis-ci.org/IFL-CAMP/iiwa_stack)
___
### Features
- Native ROSJava nodes running on the robot as a Sunrise RoboticApplication: supports ROS parameters, topics, services, etc.
- Integration of KUKA's SmartServo and PTP motions:
  - joint position, joint velocity and cartesian position control via simple ROS messages. 
  - online configuration of JointImpedance, CartesianImpedance, DesiredForce and Sine(Force)Pattern via ROS service.
  - online configuration of joint/cartesian velocity and acceleration via ROS service.
  - updates on the time left to reach the commanded destination via ROS service.
- Selection of the tool and endframe to use via ROS parameters and services.
- 'Fake' hand-guidance mode.
- NTP synchronization with a server running on the ROS master
- full MoveIt! integration
- Gazebo support

___
### Usage
__The features and usage of the stack are described in depth on its  [WIKI][8].__  
We **_strongly_** suggest to have a look at the wiki to have a better understanding of the code, both for its use and its extension.     
Do you have problems? First, please check the [**FAQs**](https://github.com/SalvoVirga/iiwa_stack/wiki/FAQ). Issues or emails are always welcome!

___
### Citation

If you use iiwa_stack for reseach, you could cite the following work. It is the first publication where it was used.

    @article{hennersperger2017towards,
      title={Towards MRI-based autonomous robotic US acquisitions: a first feasibility study},
      author={Hennersperger, Christoph and Fuerst, Bernhard and Virga, Salvatore and Zettinig, Oliver and Frisch, Benjamin and Neff, Thomas and Navab, Nassir},
      journal={IEEE transactions on medical imaging},
      volume={36},
      number={2},
      pages={538--548},
      year={2017},
      publisher={IEEE}
    }

___
### Acknowledgements
This repository takes inspiration from the work of :
- _Centro E. Piaggio_ and their [ROS interface for the KUKA LBR 4+][1]
- _Mohammad Khansari_ and his [IIWA-ROS communication inteface][2] 
- _Robert Krug_ and his [IIWA URDF and Gazebo package][7]      

Most of the original files were completely refactored though.


### Contacts

[![TUM](http://campar.in.tum.de/files/goeblr/TUM_Web_Logo_blau.png "TUM Logo")](http://www.tum.de)      
[Chair for Computer Aided Medical Procedures](http://campar.in.tum.de/)      
[Technical University of Munich](http://www.tum.de), Germany.      

<b>Salvatore Virga</b> : [salvo.virga@tum.de](mailto:salvo.virga@tum.de)      
<b>Marco Esposito</b> : [marco.esposito@tum.de](mailto:marco.esposito@tum.de) 

[1]: https://github.com/CentroEPiaggio/kuka-lwr
[2]: https://bitbucket.org/khansari/iiwa.git
[3]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/msg/IIWAMsg.msg
[4]: https://bitbucket.org/khansari/iiwa/wiki/Home
[5]: https://bitbucket.org/khansari/iiwa/src/c4578460d79d5d24f58bf94bd97fb6cb0b6f280f/JavaNode/?at=master
[6]: http://git.lcsr.jhu.edu/cgrauma1/kuka_iiwa_shared
[7]: https://github.com/rtkg/lbr_iiwa
[8]: https://github.com/SalvoVirga/iiwa_stack/wiki
