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


def DJI_map_result(tif, latest_db, lat, lon):

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

        
    """""""""""""""""""""""""""Plot reference GPS"""""""""""""""""""""""""""""

    iphone_data = """
                [
                    {"latitude": 33.65530938, "longitude": 130.67379216},
                    {"latitude": 33.65518955, "longitude": 130.67374834},
                    {"latitude": 33.65519022, "longitude": 130.67376563},
                    {"latitude": 33.6551541, "longitude": 130.67371955},
                    {"latitude": 33.6551529, "longitude": 130.67373848},
                    {"latitude": 33.65513167, "longitude": 130.67378369},
                    {"latitude": 33.65522131, "longitude": 130.67376757},
                    {"latitude": 33.65524945, "longitude": 130.67374279}
                ]
                """
    # iphone_data = json.loads(iphone_data)
    # for item in iphone_data:
    #     ax.scatter(item['longitude'], item['latitude'], color='green')
    
    rtk_data = """
        
               [
    {"latitude": 33.85320537, "longitude": 130.50163478},
    {"latitude": 33.85320805, "longitude": 130.50166540},
    {"latitude": 33.85319258, "longitude": 130.50163831},
    {"latitude": 33.85320444, "longitude": 130.50166474},
    {"latitude": 33.85318476, "longitude": 130.50162638},
    {"latitude": 33.85319179, "longitude": 130.50161724},
    {"latitude": 33.85320086, "longitude": 130.50162375},
    {"latitude": 33.85321673, "longitude": 130.50163167},
    {"latitude": 33.85321019, "longitude": 130.50162486},
    {"latitude": 33.85319408, "longitude": 130.50164424},
    {"latitude": 33.85319697, "longitude": 130.50165491},
    {"latitude": 33.85323439, "longitude": 130.50167940},
    {"latitude": 33.85321332, "longitude": 130.50167145},
    {"latitude": 33.85323229, "longitude": 130.50165465},
    {"latitude": 33.85322437, "longitude": 130.50168645},
    {"latitude": 33.85322727, "longitude": 130.50170166},
    {"latitude": 33.85320312, "longitude": 130.50170793},
    {"latitude": 33.85323214, "longitude": 130.50170030}
]


                """
    android_data = json.loads(rtk_data)
    for item in android_data:
        ax.scatter(item['longitude'], item['latitude'], color='blue',s=20)

    """""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

    """"""""""""""""""""""""""""Plot GPS from database"""""""""""""""""""""""""""
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

    """"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

    """Convert plt to image for Flask"""

    plt.tight_layout()
    buffer = io.BytesIO()
    fig.savefig(buffer, format='png')
    buffer.seek(0)
    image_data = buffer.getvalue()
    plt.close(fig)

    return track_data, image_data
