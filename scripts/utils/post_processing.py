import numpy as np
from tabulate import tabulate

def haversine(lat1, lon1, lat2, lon2):

    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    """Haversine formula"""

    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    r = 6371000  
    return c * r

def bearing(lat1, lon1, lat2, lon2):

    """Find direction of the distance error"""

    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])

    x = np.sin(lon2 - lon1) * np.cos(lat2)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(lon2 - lon1)
    initial_bearing = np.arctan2(x, y)

    initial_bearing = np.degrees(initial_bearing)
    bearing = (initial_bearing + 360) % 360
    return bearing

def calculate_errors(reference, measured):
    distance_errors = []
    bearings = []
    nearest_model = []
    for measured_key, measured_value in measured.items():
        nearest_meas_value, nearest_model_name, min_distance = find_nearest_point(measured_value, reference)
        nearest_model.append(nearest_model_name)
        distance_errors.append(min_distance)
        bearings.append(bearing(measured_value['lat'], measured_value['lon'], nearest_meas_value['lat'], nearest_meas_value['lon']))

    """find MAE MSE"""

    mae = np.mean(np.abs(distance_errors))
    mse = np.mean(np.square(distance_errors))
    rmse = np.sqrt(mse) 
    

    return {
        'nearest_model':nearest_model,
        'distance_errors': distance_errors,
        'directional_errors': bearings,
        'MAE': mae,
        'MSE': mse,
        'RMSE': rmse,
    }

def find_nearest_point(measured, ref_point):
    min_distance = float('inf')
    nearest_point = None
    for m_key, m_point in ref_point.items():
        distance = haversine(measured['lat'], measured['lon'], m_point['lat'], m_point['lon'])
        if distance < min_distance:
            min_distance = distance
            nearest_point = m_point
            nearest_model = m_key
    return nearest_point, nearest_model, min_distance

def check_duplicate(list):
    index = 0
    count = 1
    count_list = []
    for i in range(1,len(list)):
        if list[i] == list[index]: count+=1
        else: 
            if count > 1: count_list.append([list[index],count])
            index = i
            count = 1

        if i+1 == len(list) and count > 1:
            count_list.append([list[index],count])
            break
    return count_list

