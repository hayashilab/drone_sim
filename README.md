DJI Mavic Air 2S drone with ROS integration

## Gazebo Simulation
![Screenshot from 2023-06-07 15-30-15-cropped](https://github.com/hayashilab/drone_sim/assets/86349365/92824b4e-2c56-4234-9c04-dd955786a42a)

- Requirements
  - Ubuntu 20.04
  - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and set up [catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)
  - [git lfs](https://github.com/git-lfs/git-lfs/wiki/Installation)


  
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
  - Available scripts
    Script | Description
    --------- | ------------------
    Mavic_connection |  Establish connection between Mavic Air 2S and Python script, see [Drone-PC Interface](## Drone-PC Interface)
    Mavic_connection_sim |  Establish connection between Mavic Air 2S and Python script inside Gazebo simulation.
    get_sim_image |  Receive franes from drone camera inside Gazebo simulation in OpenCV format
    gazebo_spawn_object_simple |  Randomly spawn object inside Gazebo world ["beer","bowl","marble_1_5cm","plastic_cup","wood_cube_7_5cm","wooden_board"]
    
## Drone-PC Interface
  - Follow instruction to install [Rosettadrone](https://github.com/RosettaDrone/rosettadrone).
 
    1. Clone or download the repository.
        ```
        git clone https://github.com/OpenDroneMap/WebODM.git
        ```

    2. In Android Studio, select **File->New->Import Project** and navigate to the downloaded folder.
 
    3. Sign up for the DJI Developer Program at https://developer.dji.com/mobile-sdk/ and create an Application key. The package name should be sq.rogue.rosettadrone.

    4. Generate Google Maps API key using instructions at https://developers.google.com/maps/documentation/javascript/get-api-key#creating-api-keys 
 
    5. Create a new file called keys.xml in the /app/src/main/res/values/ folder, and insert the following:
        ```
        <?xml version="1.0" encoding="utf-8"?>
        <resources>
            <string name="dji_key">INSERT KEY HERE</string>
            <string name="google_key">INSERT KEY HERE</string>
        </resources>
        ```
    
    6. Run **Build->Make Project**

  

