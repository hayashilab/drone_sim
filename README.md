ROS simulation and interface with DJI Mavic Air 2S (or others).

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
    roslaunch drone_sim Beach_drone.launch
    ```
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
    Mavic_connection |  Establish connection between Mavic Air 2S and Python script, see [Drone-PC Interface](## Drone-PC Interface)
    Mavic_connection_sim |  Establish connection between Mavic Air 2S and Python script inside Gazebo simulation.
    get_sim_image |  Receive franes from drone camera inside Gazebo simulation in OpenCV format
    gazebo_spawn_object_simple |  Randomly spawn object inside Gazebo world ["beer","bowl","marble_1_5cm","plastic_cup","wood_cube_7_5cm","wooden_board"]

## Known Issues

### AUTO mode — drone does not move
`vehicle.commands.upload()` is commented out in `utils/drone_utils.py`, so the mission is never sent to SITL. Uncomment it to fix.

### Spawned objects fall through the terrain
The spawn z-height in `gazebo/SIM_model_handler.py` needs to match the terrain:
- **Hokuto**: `initial_pose.position.z = 5.0` (beach terrain sits at z=2)
- **Kyutech**: `initial_pose.position.z = 0.5`

---

## Switching Between Hokuto and Kyutech

There are **4 files** to update when switching environments. All changes are listed below.

---

### 1. `scripts/main/launch_stil_.sh` — SITL spawn coordinates

| Environment | Value |
|---|---|
| **Hokuto**  | `-l 33.853112,130.501569,0,300` |
| **Kyutech** | `-l 33.655187,130.673922,0,300` |

```bash
# Hokuto (current)
xterm -e sim_vehicle.py -v ArduCopter -f gazebo-iris -l 33.853112,130.501569,0,300 --console

# Kyutech
xterm -e sim_vehicle.py -v ArduCopter -f gazebo-iris -l 33.655187,130.673922,0,300 --console
```

---

### 2. `scripts/main/utils/drone_config.py` — Mission file

```python
# Hokuto (current)
mission_file = 'missions/hokuto_mission.txt'

# Kyutech
mission_file = 'missions/kyutech_mission.txt'
```

---

### 3. `scripts/main/SIM_drone_status.py` — Spawn area and map result

**`/spawn_objects` route** — change the area passed to `init_spawn_objects`:
```python
# Hokuto
init_spawn_objects(model_names, beach_area, model_path_prefix, object_count)

# Kyutech
init_spawn_objects(model_names, kyutech_area, model_path_prefix, object_count)
```

**`gazebo/SIM_model_handler.py`** — change spawn z-height to match terrain:
```python
# Hokuto (beach terrain sits at z=2)
initial_pose.position.z = 5.0

# Kyutech
initial_pose.position.z = 0.5
```

**`/show_map_result` route** — change tif path and coordinates:
```python
# Hokuto
model_positions, track_positions, image_data = map_result(Hokuto_tif_path, latest_db, 33.85315582, 130.50159024)

# Kyutech
model_positions, track_positions, image_data = map_result(Kyutech_tif_path, latest_db, 33.655187, 130.673922)
```

---

### 4. Launch file — Gazebo world

| Environment | Command |
|---|---|
| **Hokuto**  | `roslaunch drone_sim beach_drone.launch` |
| **Kyutech** | `roslaunch drone_sim kyutech_drone.launch` |

---

### Quick reference

| | Hokuto | Kyutech |
|---|---|---|
| Launch file | `beach_drone.launch` | `kyutech_drone.launch` |
| World | `worlds/Hokuto.world` | `worlds/KyutechField.world` |
| Mission | `missions/hokuto_mission.txt` | `missions/kyutech_mission.txt` |
| SITL coords | `33.853112, 130.501569` | `33.655187, 130.673922` |
| Map origin | `33.85315582, 130.50159024` | `33.655187, 130.673922` |
| Waypoints | 15 | 11 |

---

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

  

