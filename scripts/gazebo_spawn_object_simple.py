import rospy
import random
import time
import xml
from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from pathlib import Path
def xml_praser(model_path):
    
    f = open(model_path, 'r')
    model_xml = f.read()
    xml_parsed = xml.etree.ElementTree.fromstring(model_xml)
    model_xml = xml.etree.ElementTree.tostring(xml_parsed)
    model_xml = model_xml.decode(encoding='ascii')
    return model_xml
def delete_model(model_name):
    delete_model = rospy.ServiceProxy('%s/delete_model' % "/gazebo", DeleteModel)
    delete_model(model_name)

area = [(6, 29), (-18, 29), (-5, -20), (20, -20)]

model_names = ["beer","bowl","marble_1_5cm","plastic_cup","wood_cube_7_5cm","wooden_board"]
model_path_prefix = ""

for model_name in model_names:
    for i in range(10):
        initial_pose = Pose()
        initial_pose.position.x = random.uniform(min(point[1] for point in area), max(point[0] for point in area))
        initial_pose.position.y = random.uniform(min(point[1] for point in area), max(point[0] for point in area))

        initial_pose.position.z = 3
        model_path = model_path_prefix + model_name + "/model.sdf"
        model_xml = xml_praser(model_path)
        gazebo_interface.spawn_sdf_model_client(model_name + "_" + str(i), model_xml, rospy.get_namespace(), initial_pose, '', '/gazebo')
