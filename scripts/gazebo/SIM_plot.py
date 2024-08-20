import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
import matplotlib.pyplot as plt
import numpy as np
import rasterio
from pyproj import Proj, transform
import json
import os
import io
import matplotlib

matplotlib.use('agg')

def utm_to_latlong(east, north, crs):

    """Convert UTM coordinate to latitude and longitude"""

    utm_proj = Proj(crs)
    lat, lon = utm_proj(east, north, inverse=True)
    return lat, lon

def cartesian_to_gps(x, y, z, lat_ref, lon_ref):
    
    """Convert cartesian coordinate to latitude and longitude"""

    """Swap x and y coordinates and negate the new y coordinate"""
    x, y = -y, x

    """Convert reference latitude and longitude from degrees to radians"""
    lat_ref_rad = np.radians(lat_ref)
    lon_ref_rad = np.radians(lon_ref)

    """Calculate the absolute distance in meters"""
    offset_distance = np.sqrt(x**2 + y**2)

    """Calculate the offset in radians"""
    offset_rad = (np.pi / (180 * 60 * 1852)) * offset_distance

    """Calculate the heading in radians (assuming x is east and y is north)"""
    heading_rad = np.arctan2(x, y)

    """Calculate the target latitude"""
    target_lat_rad = np.arcsin(np.sin(lat_ref_rad) * np.cos(offset_rad) +
                               np.cos(lat_ref_rad) * np.sin(offset_rad) * np.cos(heading_rad))

    """Calculate the difference in longitude"""
    dlon = np.arctan2(np.sin(heading_rad) * np.sin(offset_rad) * np.cos(lat_ref_rad),
                      np.cos(offset_rad) - np.sin(lat_ref_rad) * np.sin(target_lat_rad))

    """Calculate the target longitude"""
    target_lon_rad = lon_ref_rad + dlon

    """Convert target latitude and longitude back to degrees"""
    target_lat = np.degrees(target_lat_rad)
    target_lon = np.degrees(target_lon_rad)

    return target_lat, target_lon, z

def get_model_gps_positions(lat_ref, lon_ref):

    """Get model reference positions from Gazebo"""

    rospy.wait_for_service('/gazebo/get_world_properties')
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world_properties = get_world_properties()
        model_names = world_properties.model_names
        
        model_gps_positions = []
        for model_name in model_names:
            rospy.wait_for_service('/gazebo/get_model_state')
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = get_model_state(model_name, '')
            position = model_state.pose.position
            lat, lon, alt = cartesian_to_gps(position.x, position.y, position.z, lat_ref, lon_ref)
            model_gps_positions.append((model_name, lat, lon, alt))
        
        return model_gps_positions
    
    except rospy.ServiceException as e:
        print("Failed getting model" % e)
        return []


def map_result(tif, latest_db, lat, lon):

    """Plot GPS from reference and database"""

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.legend()

    """Load 2D map"""
    
    map = rasterio.open(tif)
    image_data = map.read((1, 2, 3))
    image_data = np.transpose(image_data, (1, 2, 0))
    crs = map.crs
    bounds = map.bounds
    min_lon, min_lat = utm_to_latlong(bounds.left, bounds.bottom, crs)
    max_lon, max_lat = utm_to_latlong(bounds.right, bounds.top, crs)
    ax.imshow(image_data, cmap='gray', extent=[min_lon, max_lon, min_lat, max_lat])

    """""""""""""""""""""""""Plot reference GPS from Gazebo"""""""""""""""""""""""""""

    model_gps_positions = get_model_gps_positions(lat, lon)
    if model_gps_positions:
        latitudes = []
        longitudes = []
        labels = []
        colors = []
        for model_name, lat, lon, alt in model_gps_positions:
            if not model_name.startswith(("map_satellite_", "Field", "beach","gazebo","toilet","oak_tree")):
                latitudes.append(lat)
                longitudes.append(lon)
                labels.append(model_name)
                colors.append('green' if model_name == 'mavic' else 'blue')

        model_positions = {}
        for i, label in enumerate(labels):
            ax.annotate(label, (longitudes[i], latitudes[i]))
            model_positions[label] = {'lat': latitudes[i], 'lon': longitudes[i]}
        ax.scatter(np.array(longitudes), np.array(latitudes), color=colors, label='Other Objects')
    else:
        print("No objects found in the simulation.")

    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

    """"""""""""""""""""""""""""Plot GPS from database"""""""""""""""""""""""""""""""

    try:
        with open(os.path.join('garbageDB', latest_db), 'r') as file:
            data = json.load(file)

        track_positions = {}
        for track_id, track_data in data.items():
            lat, lon = track_data['positions'][0]
            track_positions[int(track_id)] = {'lat': [lat], 'lon': [lon]}
     
        for track_id in track_positions:
            track_positions[track_id]['lat'] = np.mean(track_positions[track_id]['lat'])
            track_positions[track_id]['lon'] = np.mean(track_positions[track_id]['lon'])
    
        for track_id, positions in track_positions.items():
            ax.scatter(positions['lon'], positions['lat'], label=f'Track {track_id}', color='red')

    except FileNotFoundError:
        print("File not found")

    """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

    plt.tight_layout()
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png')
    buffer.seek(0)
    image_data = buffer.getvalue()
    plt.close(fig)

    return model_positions, track_positions, image_data


