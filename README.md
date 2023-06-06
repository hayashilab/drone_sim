DJI Mavic Air 2S drone with ROS integration
-
- Requirements
  - Ubuntu 20.04
  - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and set up [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)
  
  
- Installation
  - Take a look at these tutorials to setup ardupilot, gazebo and the ardupilot gazebo plugin by [Intelligent-Quads](https://github.com/Intelligent-Quads)

      [Installing Ardupilot and MAVProxy](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)

      [Installing QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md)

      [Installing Gazebo and ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)
      
  - Clone this repository
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/hayashilab/drone_sim.git
    catkin build
    source ~/.bashrc
    ```
  - Launch the world
    ```
    roslaunch drone_sim Beach_drone.launch
    ```
  

