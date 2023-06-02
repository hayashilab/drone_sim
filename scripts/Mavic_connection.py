from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import cv2  # print(cv2.getBuildInformation()) make sure GStreammer is installed
import keyboard
import sys

def send_local_velocity( vx, vy, vz,yaw):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000011111000111, # Following ( 1 is mask OUT): yaw vel, yaw pos, force not accell, az,ay,ax, vz,vy,vx, pz,py,px 
        0, 0, 0,
        vx, vy, vz, # speed forward, right, down...
        0, 0, 0,
        0,float(yaw))  # yaw rate in radians/s...  
    vehicle.send_mavlink(msg)
    vehicle.flush()

def keyboard_cmd(speed):
    event = keyboard.read_event()
    if event.event_type == keyboard.KEY_DOWN:
        if event.name == str(8):send_local_velocity    (speed+0.1   , 0     , 0     ,0    ); command="forward"
        elif event.name == str(2):send_local_velocity  (-0.3  , 0     , 0     ,0    ); command="backward"
        elif event.name == str(6):send_local_velocity  (0     , 0.3   , 0     ,0    ); command="left"
        elif event.name == str(4):send_local_velocity  (0     , -0.3  , 0     ,0    ); command="right"
        elif event.name == str(1):send_local_velocity  (0     , 0     , 0.1   ,0    ); command="down"
        elif event.name == str(3):send_local_velocity  (0     , 0     , -0.1  ,0    ); command="up"
        elif event.name == str(7):send_local_velocity  (0     , 0     , 0     ,-0.3 ); command="cw"
        elif event.name == str(9):send_local_velocity  (0     , 0     , 0     ,0.3  ); command="ccw"
    else:send_local_velocity (0,0,0,0); command="stop"
    
    return command


# Camera connection
print("Waiting for video..")
cap_receive = cv2.VideoCapture('udpsrc port=5600 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtpjitterbuffer ! parsebin ! decodebin ! videoconvert ! appsink fps-update-interval=1000 sync=false', cv2.CAP_GSTREAMER)
if not cap_receive.isOpened():
        print('No video active over UDP')
        exit(0)
else: print("received")

# Drone connection
connection_string = 'udp:0.0.0.0:14550'
#connection_string = 'udp:192.168.1.20:14550'
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)
print("Connected!")
speed = 0


while True:
        try:
            ret,frame = cap_receive.read()
            if not ret:
                print('empty frame')
                break
            print("{}, {}".format(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon, sep='\n'))
            cv2.imshow('receive', frame)
            if cv2.waitKey(1)&0xFF == ord('q'):
                break

            # key = keyboard_cmd(speed)
            # if speed>= 0.3:speed=0.3
            # if key == "stop": speed-=0.1
            # else:speed+=0.01

            
            # print  ("_____________________________________________________________________________________")
            # print (vehicle.battery.level)
            # print (vehicle.system_status.state)
            # print (vehicle.armed)
            # print (vehicle.heading)
            # print (vehicle.groundspeed)
            # print (vehicle.gps_0.fix_type)
            # print (vehicle.gps_0.satellites_visible)
            # print (vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt)
            # print (vehicle.heading)
            # print (vehicle.groundspeed)
            # if vehicle.velocity[0] != None:print (vehicle.velocity[0])
            # if vehicle.velocity[1] != None:print (vehicle.velocity[1])
            # if vehicle.velocity[2] != None:print (vehicle.velocity[2])

            # if vehicle.home_location != None :
            #     print (vehicle.home_location.lat)
            #     print (vehicle.home_location.lon)
            #     print (vehicle.home_location.alt)

            # print (vehicle.commands.next)
            # print (vehicle.commands.count)
            # #send_local_velocity(1,0,0,0)
            # print ("_____________________________________________________________________________________")
            # var1 = 2
            # var2 = 5

            
                # "Battery: {}".format(vehicle.battery.level),
                # "Status: {}".format(vehicle.system_status.state),
                # "Armed: {}".format(vehicle.armed),
                # "Heading: {}".format(vehicle.heading),
                # "Ground Speed: {}".format(vehicle.groundspeed),
                # "GPS Type: {}".format(vehicle.gps_0.fix_type),
                # "Visible GPS: {}".format(vehicle.gps_0.satellites_visible),
                
                # "Battery: {}".format(vehicle.armed),
                # "Battery: {}".format(vehicle.armed),
                # "Battery: {}".format(vehicle.armed),
                # "Battery: {}".format(vehicle.armed),





        except(KeyboardInterrupt):
            break


print("stop")
send_local_velocity(0,0,0,0)