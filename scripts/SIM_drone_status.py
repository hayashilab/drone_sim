#!/usr/bin/python3
import warnings;warnings.filterwarnings("ignore", ".*subnormal.*")
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal

from timeit import default_timer as timer
import datetime

import json
from collections import defaultdict

import numpy as np
import cv2
import pandas as pd

import rospy
from sensor_msgs.msg import Image, CameraInfo, Range
from cv_bridge       import CvBridge, CvBridgeError

from utils.drone_utils       import *
from utils.calculation_utils import *
from utils.gui_utils         import *
from utils.drone_config      import *
from utils.post_processing   import *

from gazebo.SIM_model_handler import *
from gazebo.SIM_plot          import *

from ultralytics import YOLO
import torch

from flask import Flask, render_template, Response, stream_with_context, request,make_response
import threading

from shapely.geometry import Point, Polygon
from shapely import affinity




app = Flask(__name__,  template_folder='templates', static_folder='static')
model_positions, track_positions = None, None
deleted_ID = None

class SIMDroneStatus:
    def __init__(self):
        
        """Connect to drone"""
        
        rospy.loginfo("Connecting to vehicle on: %s" % connection_string) 
        self.vehicle = connect(connection_string, wait_ready=False)
        rospy.loginfo("Connected!")
    
        """Load YOLO model"""

        rospy.loginfo("Loading model")
        self.model = YOLO('weight/best_newdata.pt')
        rospy.loginfo("Model loaded | GPU: {}".format(torch.cuda.is_available()))
        
        
        """Parameters"""

        self.start = timer() 
        self.time_interval = time_interval
        self.time_passed = 0
        self.image_iteration = 0
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

        self.img_sub     = rospy.Subscriber("/webcam/image_raw", Image,self.camcallback)     
        self.laser_sub   = rospy.Subscriber("/sonar", Range, self.lasercallback)

        """Load mission"""
        
        self.mission = mission_file 
        rospy.loginfo("Upload mission from a file: %s" % mission_file)
        upload_mission(self.vehicle, self.mission)  
        rospy.loginfo("Mission uploaded\n")

        """Start Flask daemon thread"""

        def run_flask():
            app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
        flask_thread = threading.Thread(target=run_flask)
        flask_thread.daemon = True
        flask_thread.start()

        
    def camcallback(self, color_img_ros):

        global deleted_ID
       
        """ Callback when getting images from simulation"""   

        self.bridge = CvBridge()
        try:
            self.img = self.bridge.imgmsg_to_cv2(color_img_ros,"bgr8")
        except CvBridgeError:
            rospy.logerr("CvBawridge Error: {0}".format(self.e))                                
        self.frame = np.array(self.img)
        self.height, self.width, _ = self.frame.shape
        self.results = self.detectionYolo(self.frame)

        """If not detect, show overview GUI"""
        
        self.height_meter, self.width_meter, self.meter_per_pixel = overview_GUI(   self.laser_result,
                                                                                    self.camera_matrix,
                                                                                    self.height,
                                                                                    self.width,)
        

        self.rectangle_polygon = calculate_rectangle_coordinates(   self.vehicle.location.global_frame.lat,
                                                                    self.vehicle.location.global_frame.lon,
                                                                    self.width_meter,
                                                                    self.height_meter,
                                                                    self.vehicle.heading)
        
        
        self.rectangle_polygon = Polygon([(coord['lat'],coord['lng']) for coord in self.rectangle_polygon])
        self.rectangle_polygon = affinity.scale(self.rectangle_polygon, xfact=0.9, yfact=1)
        """If detect, show detection GUI"""

        self.m_per_p = pixel_to_meter(self.height_meter, self.height, self.width_meter, self.width)
      
        if self.detection:
            
            self.target_positions = detection_GUI_DJI(
                self.results, 
                self.vehicle.location.global_frame.lat,
                self.vehicle.location.global_frame.lon,
                self.vehicle.heading,
                self.detection, 
                self.height, 
                self.width,
                self.m_per_p,
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
                                        self.vehicle.location.global_frame.lat,
                                        self.vehicle.location.global_frame.lon,
                                        self.vehicle.heading,
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
            self.image_iteration+=1
            self.time_passed +=1


    def detectionYolo(self, img):
        self.detection = []
        Yoloresults = self.model.track(img, persist=True,verbose=False, conf = 0.5)

        if Yoloresults[0].boxes is not None and Yoloresults[0].boxes.id is not None:
            self.boxes          = Yoloresults[0].boxes.xywh.numpy()
            self.track_ids      = Yoloresults[0].boxes.id.numpy().astype(int)
            self.conf_scores    = Yoloresults[0].boxes.conf.numpy()
            self.class_ids      = Yoloresults[0].boxes.cls.numpy().astype(int)

            for box, track_id, conf, cls_id in zip(self.boxes, self.track_ids, self.conf_scores, self.class_ids):                   # Plot the tracks and labels
                    x, y, w, h = box
                    self.detection.append([x, y, x, y, conf, cls_id])
                    track = self.track_history[track_id]
                    track.append((float(x), float(y)))  

                    """Track for 30 frames"""

                    if len(track) > 30: track.pop(0)

                    points = np.array(track).reshape((-1, 1, 2)).astype(np.int32)
                    cls_name = self.classNames[cls_id]
                    x1, y1, x2, y2 = int(x - w / 2), int(y - h / 2), int(x + w / 2), int(y + h / 2)

                    cv2.polylines(img, [points], isClosed=False, color=(0, 0, 255), thickness=2)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)
                    cv2.putText (
                                    img =img,
                                    text="{} {} {:.2f}".format(cls_name, track_id, conf),
                                    org=(x1, y1 - 10),
                                    fontFace=cv2.FONT_HERSHEY_DUPLEX,
                                    fontScale=1,
                                    color=(0, 255, 0),
                                    thickness=2
                                )
        return img

    def generate_frame(self):

        """Send OpenCV to Flask"""

        while True:
            if hasattr(self, 'results'):
                _, jpeg = cv2.imencode('.jpg', self.results)
                frame = jpeg.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            else:break


    def lasercallback(self, laser_ros):

        """Get laser rangefinder distance"""

        self.laser_result = laser_ros.range

    def toggle_lines(self, show_lines):
        
        """Show detection offset lines"""

        self.show_lines = show_lines
    
                
    def gui_data(self):

        """Send Data to Flask"""

        time_passed     = self.time_passed
        latitude        = self.vehicle.location.global_frame.lat
        longitude       = self.vehicle.location.global_frame.lon
        height_meter    = self.height_meter 
        width_meter     = self.width_meter
        gps_altitude    = self.vehicle.location.global_relative_frame.alt
        laser_altitude  = np.round(self.laser_result,3)
        heading         = self.vehicle.heading
        
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
    


""""""""""""""""" Flask Route """""""""""""""""""""

@app.route('/')
def index():

    """Home page"""

    return render_template('index.html')


@app.route('/video_stream')
def video_feed():
    
    """Send video stream"""

    return Response(stream_with_context(drone_status.generate_frame()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/gui_data')
def gps_data():

    """Send data"""

    gui_data = drone_status.gui_data()  
    return gui_data

@app.route('/move_drone', methods=['POST'])
def move_drone():

    """Drone control (GUIDED mode only)"""

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

    """Force stop"""

    send_local_velocity(drone_status.vehicle, 0, 0, 0, 0)
    return 'OK'

show_lines = True
@app.route('/toggle_lines', methods=['POST'])
def toggle_lines():

    """Toggle line showing distance"""

    show_lines = request.json['show_lines']
    drone_status.toggle_lines(show_lines)
    return 'OK'

@app.route('/set_vehicle_mode', methods=['POST'])
def set_vehicle_mode():

    """Set mode (AUTO or GUIDED)"""

    data = request.get_json()
    mode = data['mode']
    drone_status.vehicle.mode = VehicleMode(mode)
    return 'OK'

@app.route('/reset_checkpoint', methods=['POST'])
def reset_checkpoint():

    """Reset checkpoint to 0"""

    drone_status.vehicle.commands.next = 0
    drone_status.vehicle.mode = VehicleMode("GUIDED")
    return 'OK'

@app.route('/spawn_objects', methods=['POST'])
def spawn_objects():

    """Spawn models in Gazebo"""

    """""Hokuto Bounding"""""
    beach_area   = [(  7, 26),
                    (-12, 27),
                    ( -3,-12), 
                    ( 17,-12)]   
    """"""""""""""""""""""""""

    """""Kyutech Bounding"""""
    kyutech_area = [( 16,  15), 
                    (-19,  12), 
                    (-17, -17), 
                    ( 18, -13)] 
    """"""""""""""""""""""""""

    model_names = ["beer", "bowl"]
    model_path_prefix = "/home/titan/ardupilot_gazebo/models/"
    object_count = 20
    init_spawn_objects(model_names, kyutech_area, model_path_prefix, object_count)
    return 'OK'

@app.route('/delete_objects', methods=['POST'])
def delete_objects():

    """Delete models in Gazebo"""

    model_names = ["beer", "bowl"]
    object_count = 20
    delete_spawned_objects(model_names, object_count)
    return 'OK'

@app.route('/show_map_result', methods=['POST'])
def show_map_result():

    global model_positions, track_positions 

    Hokuto_tif_path = "gazebo/Hokuto_orthophoto.tif"
    Kyutech_tif_path = "gazebo/Kyutech_orthophoto.tif"
    garbage_files = os.listdir('garbageDB')
    garbage_files.sort(key=lambda x: os.path.getmtime(os.path.join('garbageDB', x)), reverse=True)
    latest_db = garbage_files[0]

    """Hokuto  coordinate"""
    # lat = 33.85315582   
    # lon = 130.50159024
    """"""""""""""""""""""""

    """Kyutech coordinate"""
    lat = 33.655187         
    lon = 130.673922
    """"""""""""""""""""""""

    model_positions, track_positions, image_data = map_result(Kyutech_tif_path, latest_db,lat,lon)
    nparr = np.frombuffer(image_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)                  
    _, encoded_img = cv2.imencode('.png', img)               
    processed_image_data = encoded_img.tobytes()                     
    response = make_response(processed_image_data)
    response.headers['Content-Type'] = 'image/png'
    response.headers['Content-Disposition'] = 'inline'

    return response
    

def main():
    global drone_status, model_positions, track_positions, deleted_ID
    rospy.init_node('drone_status', anonymous=True, log_level=rospy.INFO)
    drone_status = SIMDroneStatus()
    try: rospy.spin()
    finally:

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        rospy.logdebug("\n\nPlot result:" + "\n\n{}\n\n {}\n\n {}\n\n".format(model_positions, track_positions, deleted_ID))

        """Calculate errors and format data"""

        del model_positions["mavic"]
        errors = calculate_errors(model_positions,track_positions)
        data = []
        for i,j  in enumerate(track_positions):
                data.append([j,errors['nearest_model'][i], errors['distance_errors'][i], errors['directional_errors'][i]])

        """""""""""""""Create table"""""""""""""""""

        col_names = ["Track ID","Reference object", "Distance error", "Bearing error"]
        
        """pandas"""
        df = pd.DataFrame(data, columns=col_names)
        df = df.sort_values(by="Reference object")
        
        df.to_csv(f"csvDB/garbage_table_{timestamp}.csv")
        rospy.loginfo(df)

        """tabulate"""

        # data = sorted(data, key=lambda x: x[1])
        # print(tabulate(data, headers=col_names, tablefmt="fancy_grid", showindex="always"))

        """"""""""""""""""""""""""""""""""""""""""""

        """Check for duplicate"""

        reference_objects = sorted([item[1] for item in data])
        count_list = check_duplicate(reference_objects)
        
        """Output results"""
    
        rospy.loginfo("MAE (meters): {}".format(errors['MAE']))
        rospy.loginfo("MSE (meters squared): {}".format(errors['MSE']))
        rospy.loginfo("RMSE (meters): {}".format(errors['RMSE']))
        rospy.loginfo("Count List: {}\n".format(count_list))

        """Save result as json file"""

        data = {
                "Plot result": {
                    "Model Positions": model_positions,
                    "Track Positions": track_positions,
                    "Deleted IDs": deleted_ID
                },
                "Errors": {
                    "MAE (meters)": errors['MAE'],
                    "MSE (meters squared)": errors['MSE'],
                    "RMSE (meters)": errors['RMSE']
                },
                "Count List": count_list
                }
        
        with open(f"resultDB/garbage_table_{timestamp}.json", "w") as file:
            json.dump(data, file, indent=4) 


        
        rospy.loginfo("\nShutting down node...\n\n\n")


if __name__ == '__main__':
    main() 