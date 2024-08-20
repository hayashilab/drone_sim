ROS simulation and interface with DJI Air 2S (or others).

## Gazebo Simulation

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
    echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/drone_sim/models' >> ~/.bashrc
    source ~/.bashrc
    ```
  - Launch the world
    ```
    roslaunch drone_sim  beach_drone.launch
    ```
  - Launch SITL
    ```
    cd ~/ardupilot/ArduCopter/
    sim_vehicle.py -v ArduCopter -f gazebo-iris --console
    ```
  - 
    ![Screenshot from 2023-06-07 15-30-15-cropped](https://github.com/hayashilab/drone_sim/assets/86349365/92824b4e-2c56-4234-9c04-dd955786a42a)
    
  - Terminal drone command
    ```
    mode guided     #switch to GUIDED mode
    arm throttle    #arm the drone
    takeoff z       #take off to z meter
    position x y z  #move to position x y z relative to previous position
    set yaw r 0 1   #rotate r degree relative to previous position
    ```
  
  - Available scripts
    Script | Description
    --------- | ------------------
    DJI_drone_status |  Establish connection between Mavic Air 2S and Python script, see [Drone-PC Interface](## Drone-PC Interface)
    SIM_drone_status |  Establish connection between Mavic Air 2S and Python script inside Gazebo simulation.
    tools/convert_plan | Convert flight plan format from QGroundControl to MAVLink
    tools/extract_video_frame | Extract images from video
    tools/get_sim_image | Receive drone camera video from simulation (for testing)
    tools/save_rosbag_video | Save rosbag video as a file
    
    
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

  

