#!/usr/bin/python3
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal

from timeit import default_timer as timer
import datetime

import json
from collections import defaultdict

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image, CameraInfo, Range
from std_msgs.msg    import Float64, Float32
from cv_bridge       import CvBridge, CvBridgeError

from utils.drone_utils       import *
from utils.calculation_utils import *
from utils.gui_utils         import *
from utils.drone_config      import *


from ultralytics import YOLO

from gazebo.SIM_model_handler import *
from gazebo.DJI_plot import *

from flask import Flask, render_template, Response, stream_with_context, request,make_response
import threading

from shapely.geometry import Point, Polygon
import torch

app = Flask(__name__, template_folder='templates', static_folder='static')
model_positions, track_positions = None, None
deleted_ID = None

class DJIDroneStatus:
    def __init__(self):
        
        """Connect to drone"""
        
        print("\nConnecting to vehicle on: %s" % connection_string) 
        #self.vehicle = connect(connection_string, wait_ready=False)
        print("Connected!")
        
        """Load YOLO model"""

        print("Loading model")
        self.model = YOLO(model_path)
        rospy.loginfo("Model loaded | GPU: {}".format(torch.cuda.is_available()))
        print("Model loaded")
        
        """Parameters"""
    
        self.start = timer() 
        self.time_interval = time_interval
        self.time_passed = 0
        self.cvtextsize = cvtextsize
        self.cvtextbold = cvtextbold
        self.cvfont = getattr(cv2, cvfont)
        self.camera_matrix = camera_matrix
        self.show_lines = True

        """YOLO parameters"""
        
        self.classNames = classNames 
        self.detectClassNames = detectClassNames
        self.list_cls_detectClasses = [self.classNames.index(name) for name in self.detectClassNames]
        self.track_history = defaultdict(list)
         
        """Detection parameters"""

        self.garbage_database_raw = {}
        self.garbage_database_raw_all = {}
        self.garbage_database_filtered = {}
        self.garbage_database_temp = {}
        self.garbage_database_final = {}
        self.delete_ID_all = []
        self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"garbageDB/garbage_database_{self.timestamp}.json"

        """Subscribe to sensors"""
        
        rospy.Subscriber("/camera/image_raw",       Image,   self.camcallback) 
        rospy.Subscriber('/sensor/yaw',             Float32, self.sensorcallback)
        rospy.Subscriber('/sensor/latitude',        Float64, self.sensorcallback)
        rospy.Subscriber('/sensor/longitude',       Float64, self.sensorcallback)
        rospy.Subscriber('/sensor/altitude',        Float32, self.sensorcallback)
        rospy.Subscriber('/sensor/ground_altitude', Float32, self.sensorcallback)
            
        """Load mission"""
        
        self.mission = mission_file 
        #upload_mission(self.vehicle, self.mission)  
        print("\nStarting mission\n")

        """Start Flask"""

        self.flask_thread = threading.Thread(target=self.run_flask) 
        self.flask_thread.daemon = True
        self.flask_thread.start()

    
    def toggle_lines(self, show_lines):
        
        """Show detection offset lines"""

        self.show_lines = show_lines

    def sensorcallback(self, msg):
        
        """Callback when receive sensors data"""

        if msg._type == 'std_msgs/Float32':
            if msg._connection_header['topic'] == '/sensor/yaw':
                self.DJI_yaw = msg.data
            elif msg._connection_header['topic'] == '/sensor/altitude':
                self.DJI_altitude = msg.data
            elif msg._connection_header['topic'] == '/sensor/relative_altitude':
                self.DJI_relative_altitude = msg.data
        elif msg._type == 'std_msgs/Float64':
            if msg._connection_header['topic'] == '/sensor/latitude':
                self.DJI_latitude = msg.data
            elif msg._connection_header['topic'] == '/sensor/longitude':
                self.DJI_longitude = msg.data

    def camcallback(self, color_img_ros):
        
        global deleted_ID

        """ Callback when getting images from drone"""        

        self.bridge = CvBridge()
        try:
            self.img = self.bridge.imgmsg_to_cv2(color_img_ros,"bgr8")
        except CvBridgeError:
            rospy.logerr("CvBawridge Error: {0}".format(self.e))                                
        self.frame = np.array(self.img)
        self.height, self.width, _ = self.frame.shape
        self.results = self.detectionYolo(self.frame)

        self.laser_result = 5.0
        
        self.height_meter, self.width_meter, self.meter_per_pixel = overview_GUI(   self.laser_result,
                                                                                    self.camera_matrix,
                                                                                    self.height,
                                                                                    self.width,)

        self.rectangle_polygon = calculate_rectangle_coordinates(   self.DJI_latitude,
                                                                    self.DJI_longitude,
                                                                    self.width_meter,
                                                                    self.height_meter,
                                                                    self.DJI_yaw)
        
        self.rectangle_polygon = Polygon([(coord['lat'],coord['lng']) for coord in self.rectangle_polygon])
      
        self.m_per_p = pixel_to_meter(self.height_meter, self.height, self.width_meter, self.width)

        if self.detection:
            self.garbage_database_temp = {}
            self.target_positions = detection_GUI_DJI(
                self.results, 
                self.DJI_latitude,
                self.DJI_longitude,
                self.DJI_yaw,
                self.detection, 
                self.height, 
                self.width,
                pixel_to_meter(self.height_meter, self.height, self.width_meter, self.width),
                self.cvfont, 
                self.cvtextsize, 
                self.cvtextbold, 
                self.show_lines)
            
            """Add ID and position of tracked object to database_raw"""

            self.garbage_database_temp = {}
            if self.target_positions and len(self.target_positions) == len(self.track_ids):           
                for i in range(len(self.track_ids)):
                    if i < len(self.target_positions):
                        positions = [(float(self.target_positions[i][0]), float(self.target_positions[i][1]))]
                        self.entry =    {
                                        "track_id": int(self.track_ids[i]),
                                        "positions": positions
                                        }
                        
                        if self.rectangle_polygon.contains(Point(positions[0][0], positions[0][1])):
                            self.garbage_database_raw_all[self.track_ids[i]] = self.entry
                            self.garbage_database_raw[self.track_ids[i]] = self.entry
                            self.garbage_database_temp[self.track_ids[i]] = self.entry
            else:print("Detection mismatch or no items detected")

            """database_filtered includes every recorded ID but exclude ID that are currently inside polygon """
            
            self.delete_ID = []
            for track_id, entry in self.garbage_database_raw.items():
                point = Point(entry['positions'][0][0], entry['positions'][0][1])
                if self.rectangle_polygon.contains(point): 
                    #print("IN  {} {}".format(track_id,point))
                    if track_id in self.garbage_database_filtered and track_id in self.garbage_database_raw: 
                        self.delete_ID.append(track_id)
                        self.delete_ID_all.append(track_id)
                        #print("deleted {}".format(track_id))
                else: self.garbage_database_filtered[track_id] = entry
                    #print("OUT {} {}".format(track_id,point))

            """If recorded ID in database_raw show up again inside polygon, delete that ID"""
            
            if self.delete_ID:            
                for track_id in self.delete_ID:  
                    if self.rectangle_polygon.contains(Point(self.garbage_database_raw[track_id]["positions"][0][0], self.garbage_database_raw[track_id]["positions"][0][1])):

                        del self.garbage_database_filtered[track_id]
                        del self.garbage_database_raw[track_id]

            """"Show the deleted ID in video"""

            deleted_ID = deleted_ID_GUI(self.delete_ID_all,
                                        self.garbage_database_raw_all,
                                        self.DJI_latitude,
                                        self.DJI_longitude,
                                        self.DJI_yaw,
                                        self.m_per_p,
                                        self.width,
                                        self.height,
                                        self.results,
                                        0.5)
            
            
            
            """Save the processed database"""
            
            self.garbage_database_final = {**self.garbage_database_filtered, **self.garbage_database_temp}
            self.garbage_database_final = {int(key): value for key, value in self.garbage_database_final.items()}

            with open(self.filename, "w") as file:
                json.dump(self.garbage_database_final, file, indent=4)

        """Timer in second"""

        self.end = timer()
        self.time_total = np.round(self.end-self.start,2)

        if self.time_total >= self.time_interval:
            self.start = timer()
            self.time_passed +=1
         

    def detectionYolo(self, img):
        self.detection = []
        Yoloresults = self.model.track(img, persist=True,verbose=False, tracker="bytetrack.yaml")

        if Yoloresults[0].boxes is not None and Yoloresults[0].boxes.id is not None:
            self.boxes = Yoloresults[0].boxes.xywh.cpu().numpy()
            self.track_ids = Yoloresults[0].boxes.id.cpu().numpy().astype(int)
            self.conf_scores = Yoloresults[0].boxes.conf.cpu().numpy()
            self.class_ids = Yoloresults[0].boxes.cls.cpu().numpy().astype(int)

            for box, track_id, conf, cls_id in zip(self.boxes, self.track_ids, self.conf_scores, self.class_ids):                   # Plot the tracks and labels
                    x, y, w, h = box
                    self.detection.append([x, y, x, y, conf, cls_id])
                    track = self.track_history[track_id]
                    track.append((float(x), float(y)))  

                    if len(track) > 30:  
                        track.pop(0)
                    points = np.array(track).reshape((-1, 1, 2)).astype(np.int32)
                    cv2.polylines(img, [points], isClosed=False, color=(0, 0, 255), thickness=2)
                    cls_name = self.classNames[cls_id]
                    x1, y1, x2, y2 = int(x - w / 2), int(y - h / 2), int(x + w / 2), int(y + h / 2)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)
                    cv2.putText(
                                    img=img,
                                    text="{} {} {:.2f}".format(cls_name, track_id, conf),
                                    org=(x1, y1 - 10),
                                    fontFace=cv2.FONT_HERSHEY_DUPLEX,
                                    fontScale=1,
                                    color=(0, 255, 0),
                                    thickness=2
                                )
        return img
        

    def run_flask(self):
        app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

    def generate_frame(self):

        """Send OpenCV to Flask"""

        while True:
            if hasattr(self, 'results'):
                _, jpeg = cv2.imencode('.jpg', self.results)
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            else:break
                
    def gui_data(self):

        """Send Data to Flask"""
        
        time_passed = self.time_passed

        if math.isnan(self.DJI_latitude):latitude = "No GPS"
        else:latitude = self.DJI_latitude

        if math.isnan(self.DJI_longitude):longitude = "No GPS"
        else:longitude = self.DJI_longitude

        height_meter = self.height_meter
        width_meter = self.width_meter
        gps_altitude = self.DJI_altitude
        laser_altitude = np.round(self.laser_result,3)
        heading = np.round(self.DJI_yaw,3)
        last_positions = {}
    
        for track_id, entry in self.garbage_database_final.items():
            if entry["positions"]:last_positions[track_id] = entry["positions"][-1]

        target_positions = [f"{pos[0]},{pos[1]}" for pos in last_positions.values()]
        target_positions = "; ".join(target_positions)
                
        data =  {
                    'time_passed': time_passed,
                    'latitude': latitude, 
                    'longitude': longitude,
                    'height_meter': height_meter,
                    'width_meter': width_meter,
                    'gps_altitude': gps_altitude,
                    'laser_altitude': laser_altitude,
                    'heading':heading,
                    'target_position':target_positions
                }
     
        return data

@app.route('/')
def index():

    """Video streaming home page."""

    return render_template('index.html')


@app.route('/video_stream')
def video_feed():
    return Response(stream_with_context(drone_status.generate_frame()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gui_data')
def gps_data():
    gui_data = drone_status.gui_data()  
    return gui_data

@app.route('/move_drone', methods=['POST'])
def move_drone():
    direction = request.json['direction']
    if direction == 'forward':
        send_local_velocity(drone_status.vehicle, 0.2, 0, 0, 0)
    elif direction == 'left':
        send_local_velocity(drone_status.vehicle, 0, -0.2, 0, 0)
    elif direction == 'right':
        send_local_velocity(drone_status.vehicle, 0, 0.2, 0, 0)
    elif direction == 'backward':
        send_local_velocity(drone_status.vehicle, -0.2, 0, 0, 0)
    elif direction == 'ccw':
        send_local_velocity(drone_status.vehicle, 0, 0, 0, -0.2)
    elif direction == 'cw':
        send_local_velocity(drone_status.vehicle, 0, 0, 0, 0.2)
    elif direction == 'up':
        send_local_velocity(drone_status.vehicle, 0, 0, -0.3, 0)
    elif direction == 'down':
        send_local_velocity(drone_status.vehicle, 0, 0, 0.3, 0)
    return 'OK'

@app.route('/stop_drone', methods=['POST'])
def stop_drone():
    send_local_velocity(drone_status.vehicle, 0, 0, 0, 0)
    return 'OK'

show_lines = True
@app.route('/toggle_lines', methods=['POST'])
def toggle_lines():
    show_lines = request.json['show_lines']
    drone_status.toggle_lines(show_lines)
    return 'OK'

@app.route('/set_vehicle_mode', methods=['POST'])
def set_vehicle_mode():
    data = request.get_json()
    mode = data['mode']
    drone_status.vehicle.mode = VehicleMode(mode)
    return 'OK'

@app.route('/reset_checkpoint', methods=['POST'])
def reset_checkpoint():

    """Reset drone mapping checkpoint to 0"""

    drone_status.vehicle.commands.next = 0
    drone_status.vehicle.mode = VehicleMode("GUIDED")
    return 'OK'

@app.route('/spawn_objects', methods=['POST'])
def spawn_objects():

    """Spawn simulated models in Gazebo sim"""

    beach_area = [(7, 26), (-12, 27), (-3, -12), (17, -12)]   #Hokuto
    kyutech_area = [(16, 15), (-19, 12), (-17, -17), (18, -13)]   #Kyutech
    model_names = ["beer", "bowl", "wooden_board"]
    model_path_prefix = "/home/titan/ardupilot_gazebo/models/"
    object_count = 20
    init_spawn_objects(model_names, kyutech_area, model_path_prefix, object_count)
    return 'OK'

@app.route('/delete_objects', methods=['POST'])
def delete_objects():

    """Delete simulated models in Gazebo sim"""

    model_names = ["beer", "bowl", "wooden_board"]
    object_count = 20
    delete_spawned_objects(model_names, object_count)
    return 'OK'

@app.route('/show_map_result', methods=['POST'])
def show_map_result():

    global track_positions

    Hokuto_tif_path = "gazebo/Hokuto_orthophoto.tif"
    Kyutech_tif_path = "gazebo/Kyutech_orthophoto.tif"
    garbage_files = os.listdir('garbageDB')
    garbage_files.sort(key=lambda x: os.path.getmtime(os.path.join('garbageDB', x)), reverse=True)
    latest_db = garbage_files[0]

    """Hokuto  coordinate"""
    lat = 33.85315582   
    lon = 130.50159024
    """"""""""""""""""""""""

    """Kyutech coordinate"""
    # lat = 33.655187         
    # lon = 130.673922
    """"""""""""""""""""""""

    track_positions, image_data = DJI_map_result(Hokuto_tif_path, latest_db,lat,lon)
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    non_white_indices = np.where(np.any(gray < 255, axis=1))[0]      
    top = non_white_indices[0]                                      
    bottom = non_white_indices[-1]
    cropped_img = img[top:bottom+1, :]                               
    _, encoded_img = cv2.imencode('.png', cropped_img)               
    processed_image_data = encoded_img.tobytes()                     
    response = make_response(processed_image_data)
    response.headers['Content-Type'] = 'image/png'
    response.headers['Content-Disposition'] = 'inline'
    return response
    
def main():
    global drone_status, track_positions,deleted_ID
    rospy.init_node('drone_status', anonymous=True)
    drone_status = DJIDroneStatus()
    try: rospy.spin()
    finally:
        print("\n\n{}\n\n ".format(deleted_ID))
        rospy.loginfo("Shutting down node...\n"+"_"*30)

if __name__ == '__main__':
    main() 