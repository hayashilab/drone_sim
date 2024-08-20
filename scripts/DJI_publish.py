import socket
import struct
import numpy as np
import cv2
import asyncio
import logging
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32, Float64


rospy.init_node('video_stream_node', anonymous=True)

image_pub             = rospy.Publisher('/camera/image_raw',       Image  ,queue_size=10)
yaw_pub               = rospy.Publisher('/sensor/yaw',             Float32,queue_size=10)
latitude_pub          = rospy.Publisher('/sensor/latitude',        Float64,queue_size=10)
longitude_pub         = rospy.Publisher('/sensor/longitude',       Float64,queue_size=10)
altitude_pub          = rospy.Publisher('/sensor/altitude',        Float32,queue_size=10)
relative_altitude_pub = rospy.Publisher('/sensor/ground_altitude', Float32,queue_size=10)

async def receive_frame(client_socket, loop):
    try:
        start_time = time.time()
        width_data = await loop.sock_recv(client_socket, 4)
        height_data = await loop.sock_recv(client_socket, 4)

        if not width_data or not height_data:
            rospy.logerr("Incomplete frame size data received")
            return None, start_time  
        width = struct.unpack('<i', width_data)[0]
        height = struct.unpack('<i', height_data)[0]

        timestamp_data = await loop.sock_recv(client_socket, 8)
        yaw_data = await loop.sock_recv(client_socket, 4)
        yaw = struct.unpack('<f', yaw_data)[0]

        latitude_data = await loop.sock_recv(client_socket, 8)
        latitude = struct.unpack('<d', latitude_data)[0]

        longitude_data = await loop.sock_recv(client_socket, 8)
        longitude = struct.unpack('<d', longitude_data)[0]

        altitude_data = await loop.sock_recv(client_socket, 4)
        altitude = struct.unpack('<f', altitude_data)[0]

        relative_altitude_data = await loop.sock_recv(client_socket, 4)
        relative_altitude = struct.unpack('<f', relative_altitude_data)[0]

        size_data = await loop.sock_recv(client_socket, 4)
        size = struct.unpack('<i', size_data)[0]

        frame_data = b''
        while len(frame_data) < size:
            new_data = await loop.sock_recv(client_socket, size - len(frame_data))
            if not new_data:
                rospy.logerr("Disconnected during frame reception")
                return None, start_time  # Include the start time even in case of error
            frame_data += new_data

        yuv_frame = np.frombuffer(frame_data, dtype=np.uint8)
        yuv_frame = yuv_frame.reshape((height * 3 // 2, width))
        rgb_frame = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2BGR_I420)

        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
        image_pub.publish(ros_image)
        yaw_pub.publish(yaw)
        latitude_pub.publish(latitude)
        longitude_pub.publish(longitude)
        altitude_pub.publish(altitude)
        relative_altitude_pub.publish(relative_altitude)

        return (width, height, yaw, latitude, longitude, altitude, relative_altitude, rgb_frame), start_time

    except Exception as e:
        rospy.logerr("Error receiving frame")
        return None, time.time()  
    
async def main():
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server_socket.bind(("192.168.50.162", 6000))
    server_socket.listen(1)

    rospy.loginfo("Server is waiting for a connection...")
    client_socket, address = server_socket.accept()
    rospy.loginfo(f"Connected to {address}")

    loop = asyncio.get_running_loop()
    
    try:
        while not rospy.is_shutdown():
            frame_info, frame_received_time = await receive_frame(client_socket, loop)
            if frame_info is None:
                rospy.logwarn("Failed to receive frame")
                continue  # Skip to next iteration
            #ospy.loginfo(f"Frame published with timestamp: {frame_received_time}")
    finally:
        client_socket.close()
        server_socket.close()
        rospy.loginfo("Server shutdown")

if __name__ == "__main__":
    try:
        rospy.loginfo("Starting")
        asyncio.run(main())
    except rospy.ROSInterruptException:
        pass