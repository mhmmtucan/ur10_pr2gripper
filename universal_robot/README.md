universal_robot
======

[ROS-Industrial](http://wiki.ros.org/Industrial) universal_robot meta-package. See the [ROS wiki](http://wiki.ros.org/universal_robot) page for compatibility information and other more information.

To make the software work you need to install python interpreter also you need to install python-opencv, imutils, scipy, pyassimp(3.3) and moveit modules.

To get the object, container and robot models you can use the shell script that is called setup.sh, it can be found in the github repository. You need to open up a terminal in the same folder as setup.sh file. The script will move our world and models to necessary folders.

__Usage with Gazebo Simulation__  
To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur10.launch```

__MoveIt! with a simulated robot__  
You can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```

To run and visualize the software you can run the run_ur10.sh shell script. It will open up three terminal tabs and will run necessary terminal commands to launch the gazebo and rviz. After running the script you need to use ‘rosrun ur_detector detector.py’ and ‘rosrun ur_controller ur10_controller.py’ in that order, otherwise it might not work as it should.

Run those lines in seperate terminal window or use run_ur10.sh

```roslaunch ur_gazebo ur10.launch```

```roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true use_gui:=false```

```roslaunch ur10_moveit_config moveit_rviz.launch config:=true```



