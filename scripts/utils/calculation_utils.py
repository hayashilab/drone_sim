import numpy as np
from geopy import Point
from geopy.distance import distance


def calculate_fov(res_h, res_w, fx, fy):

    """Calculate field of view from calibrated camera matrix"""

    fovY = np.degrees(2 * np.arctan(res_h / (2 * fy)))
    fovX = np.degrees(2 * np.arctan(res_w / (2 * fx)))

    return fovY, fovX

def calculate_area(alt, fovY, fovX):

    """Calculate frame height and width in meters"""

    height_meter = 2 * (np.tan(np.radians(fovY / 2)) * alt)
    width_meter = 2 * (np.tan(np.radians(fovX / 2)) * alt)

    return np.round(height_meter, 5), np.round(width_meter, 5)

def pixel_to_meter(height_meter, height_pixel, width_meter, width_pixel):

    """Calculate how many meter per one pixel"""

    meter_per_pixel_h = height_meter / height_pixel
    meter_per_pixel_w = width_meter / width_pixel
    average_meter_per_pixel = (meter_per_pixel_h + meter_per_pixel_w) / 2

    return average_meter_per_pixel

def calculate_target_position(base_lat, base_lon, offset_east, offset_north, tc):

    """Calculate target GPS"""

    base_lat_rad = np.radians(base_lat)
    base_lon_rad = np.radians(base_lon)
    tc = np.radians(tc)

    offset_distance = np.sqrt(offset_east**2 + offset_north**2)
    offset_rad = (np.pi / (180 * 60 * 1852)) * offset_distance

    """
        Calculate target latitude and longitude using:
          Great Circle Navigation Formulae > Lat/lon given radial and distance
        http://www.edwilliams.org/avform147.htm#LL
    """
    target_lat_rad = np.arcsin(np.sin(base_lat_rad) * np.cos(offset_rad) +
                               np.cos(base_lat_rad) * np.sin(offset_rad) * np.cos(tc))

    
    dlon = np.arctan2(np.sin(tc) * np.sin(offset_rad) * np.cos(base_lat_rad),
                      np.cos(offset_rad) - np.sin(base_lat_rad) * np.sin(target_lat_rad))
    target_lon_rad = base_lon_rad + dlon

    target_lat = np.degrees(target_lat_rad)
    target_lon = np.degrees(target_lon_rad)

    return target_lat, target_lon


def calculate_rectangle_coordinates(latitude, longitude, width, height, heading):
    # Constants for degree to radian conversion and for meter per degree
    DEGREE_TO_RADIAN = np.pi / 180
    METERS_PER_DEGREE_LATITUDE = 111111  # Approximate meters per degree of latitude

    # Convert height and width from meters to degrees
    height_degrees = height / METERS_PER_DEGREE_LATITUDE
    width_degrees = width / (METERS_PER_DEGREE_LATITUDE * np.cos(latitude * DEGREE_TO_RADIAN))

    # Calculate half dimensions
    half_height = height_degrees / 2
    half_width = width_degrees / 2

    # Heading converted to radians
    radians = heading * DEGREE_TO_RADIAN
    sin_heading = np.sin(radians)
    cos_heading = np.cos(radians)

    # Displacement from center to corners
    dx1 = -half_width * cos_heading - half_height * sin_heading
    dy1 = half_width * sin_heading - half_height * cos_heading
    dx2 = half_width * cos_heading - half_height * sin_heading
    dy2 = -half_width * sin_heading - half_height * cos_heading
    dx3 = half_width * cos_heading + half_height * sin_heading
    dy3 = -half_width * sin_heading + half_height * cos_heading
    dx4 = -half_width * cos_heading + half_height * sin_heading
    dy4 = half_width * sin_heading + half_height * cos_heading

    # Calculating the coordinates of the four corners
    rectangle_coords = [
        {'lat': latitude + dy1, 'lng': longitude + dx1},
        {'lat': latitude + dy2, 'lng': longitude + dx2},
        {'lat': latitude + dy3, 'lng': longitude + dx3},
        {'lat': latitude + dy4, 'lng': longitude + dx4}
    ]

    return rectangle_coords

def is_point_inside_rectangle(point, rectangle_polygon):
    lat, lng = point
    min_lat = min(coord['lat'] for coord in rectangle_polygon)
    max_lat = max(coord['lat'] for coord in rectangle_polygon)
    min_lng = min(coord['lng'] for coord in rectangle_polygon)
    max_lng = max(coord['lng'] for coord in rectangle_polygon)
    return min_lat <= lat <= max_lat and min_lng <= lng <= max_lng
