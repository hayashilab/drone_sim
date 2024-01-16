from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import cv2
import keyboard


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


connection_string = 'udp:0.0.0.0:14550'
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=False)
print("Connected!")



while True:
        try:
            send_local_velocity (0,0,0,0)
            # event = keyboard.read_event()
            # if event.event_type == keyboard.KEY_DOWN and event.name == '8':
            #     print('space was pressed,forward')
            #     send_local_velocity(0.3,0,0,0)
            # elif event.event_type == keyboard.KEY_DOWN and event.name == '2':
            #     print('space was pressed,forward')
            #     send_local_velocity(-0.3,0,0,0)
            # elif event.event_type == keyboard.KEY_DOWN and event.name == '2':
            #     print('space was pressed,forward')
            #     send_local_velocity(-0.3,0,0,0)
            # elif event.event_type == keyboard.KEY_DOWN and event.name == '2':
            #     print('space was pressed,forward')
            #     send_local_velocity(-0.3,0,0,0)
            # else:
            #     send_local_velocity(0,0,0,0)
            # print("INFORMATION", 
            # "----",
            # "Battery: {} |".format(vehicle.battery.level),
            # "Status: {} |".format(vehicle.system_status.state),
            # "Armed: {} |".format(vehicle.armed),
            # "Heading: {} |".format(vehicle.heading),
            # "Ground Speed: {} |".format(vehicle.groundspeed),
            # "GPS Type: {} |".format(vehicle.gps_0.fix_type),
            # "Visible GPS: {} |".format(vehicle.gps_0.satellites_visible),
            # "Global Lat, Long, Alt: {}{}{} |".format(vehicle.location.global_frame.lat,vehicle.location.global_frame.lon,vehicle.location.global_frame.alt, sep='\n'))
            # "Battery: {}".format(vehicle.armed),
            # "Battery: {}".format(vehicle.armed),
            # "Battery: {}".format(vehicle.armed),
            # "Battery: {}".format(vehicle.armed),
        
        except(KeyboardInterrupt):
            break
print("stop")
send_local_velocity(0,0,0,0)
