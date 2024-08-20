import rospy
import random
import time
import xml
import argparse
from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from pathlib import Path
from shapely.geometry import Point, Polygon

def xml_parser(model_path):
    f = open(model_path, 'r')
    model_xml = f.read()
    xml_parsed = xml.etree.ElementTree.fromstring(model_xml)
    model_xml = xml.etree.ElementTree.tostring(xml_parsed)
    model_xml = model_xml.decode(encoding='ascii')
    return model_xml

def delete_model(model_name):
    delete_model = rospy.ServiceProxy('%s/delete_model' % "/gazebo", DeleteModel)
    delete_model(model_name)

def is_inside_polygon(x, y, polygon):
    point = Point(x, y)
    polygon = Polygon(polygon)
    return polygon.contains(point)

def init_spawn_objects(model_names, area, model_path_prefix, count):
    
    for model_name in model_names:
        for i in range(count):
            initial_pose = Pose()
            while True:
                x = random.uniform(min(point[0] for point in area), max(point[0] for point in area))
                y = random.uniform(min(point[1] for point in area), max(point[1] for point in area))
                if is_inside_polygon(x, y, area):
                    initial_pose.position.x = x
                    initial_pose.position.y = y
                    initial_pose.position.z = 0.5    # 5 for Hokuto, 0.5 for Kyutech
                    break
            model_path = model_path_prefix + model_name + "/model.sdf"
            if model_name == "beer": 
                initial_pose.orientation.x = 1.57
                initial_pose.orientation.y = 1.57
                initial_pose.orientation.w = 1.00
            model_xml = xml_parser(model_path)
            
            gazebo_interface.spawn_sdf_model_client(model_name + "_" + str(i), model_xml, rospy.get_namespace(), initial_pose, '', '/gazebo')

def delete_spawned_objects(model_names, count):
    for model_name in model_names:
        for i in range(count):
            delete_model(model_name + "_" + str(i))

