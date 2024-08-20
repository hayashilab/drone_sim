import cv2
import numpy as np
from utils.calculation_utils import *
from utils.post_processing import *
def overview_GUI(laser_result, camera_matrix, height, width):

    """GUI when not detecting anything"""

    fovY, fovX                  = calculate_fov (height, width, camera_matrix[0][0], camera_matrix[1][1])
    height_meter, width_meter   = calculate_area(laser_result, fovY, fovX)
    meter_per_pixel             = pixel_to_meter(height_meter, height, width_meter, width)

    return height_meter, width_meter, meter_per_pixel


def detection_GUI(results, vehicle, detection, height, width, meter_per_pixel, cvfont, cvtextsize, cvtextbold, show_lines):
    target_positions = []

    """GUI when detecting something for SIM drone"""
    
    for det in detection:
        x1, y1, x2, y2, conf, cls = det
        centerx = int((x1 + x2) / 2)
        centery = int((y1 + y2) / 2)
        ypixel_to_target, xpixel_to_target = height // 2 - centery, centerx - width // 2
        ymeter_to_target, xmeter_to_target = meter_per_pixel * ypixel_to_target, meter_per_pixel * xpixel_to_target



        north_x = int(width // 2 + int(2 / meter_per_pixel) * np.cos(np.radians(vehicle.heading + 90)))
        north_y = int(height // 2 - int(2 / meter_per_pixel) * np.sin(np.radians(vehicle.heading + 90)))
        
        """Offset distance from center to north point"""

        dx_north = north_x - width // 2
        dy_north = north_y - height // 2

        """Offset distance from center to target point"""

        dx_target = centerx - width // 2
        dy_target = centery - height // 2

        """Find angle between 2 vectors"""
   
        angle_radians = np.arctan2(dy_target * dx_north - dy_north * dx_target, dx_target * dx_north + dy_target * dy_north)
        angle_degrees = np.degrees(angle_radians)

        """Normalize"""
        
        angle_degrees = (angle_degrees + 360) % 360

        """"""""""""""""""
        
        target_point_latitude, target_point_longitude = calculate_target_position(
            vehicle.location.global_frame.lat,
            vehicle.location.global_frame.lon,
            abs(xmeter_to_target),
            abs(ymeter_to_target),
            angle_degrees
        )

        target_positions.append((target_point_latitude, target_point_longitude))

        if show_lines:
            # Draw north direction indecator (2 meters)
            
            cv2.line(results, (width // 2, height // 2), (north_x, north_y), (255, 255, 0), 2)
           
            # Draw lines from center to target
            cv2.line(results, (centerx, centery), (width // 2, height // 2), (0, 255, 0), 2)
            cv2.line(results, (centerx, centery), (centerx, height // 2), (255, 0, 0), 2)
            cv2.line(results, (centerx, height // 2), (width // 2, height // 2), (0, 0, 255), 2)

            # Put text for pixel distance
            cv2.putText(results,
                        str(xpixel_to_target),
                        ((centerx + width // 2) // 2, 
                        height // 2 - 5),
                        cvfont, cvtextsize, 
                        (0, 0, 255), 
                        cvtextbold, 
                        cv2.LINE_AA)
            
            cv2.putText(results, 
                        str(ypixel_to_target), 
                        (centerx + 5, 
                        (centery + height // 2) // 2), 
                        cvfont, 
                        cvtextsize, 
                        (255, 0, 0), 
                        cvtextbold, 
                        cv2.LINE_AA)

            # Put text for meter distance
            cv2.putText(results, 
                        str(np.round(xmeter_to_target, 3)), 
                        ((centerx + width // 2) // 2, 
                        height // 2 + 40), 
                        cvfont, 
                        cvtextsize, 
                        (0, 0, 255), 
                        cvtextbold, 
                        cv2.LINE_AA)
            
            cv2.putText(results, 
                        str(np.round(ymeter_to_target, 3)), 
                        (centerx - 100, 
                        (centery + height // 2) // 2), 
                        cvfont, cvtextsize, 
                        (255, 0, 0), 
                        cvtextbold, 
                        cv2.LINE_AA)


    return target_positions


def detection_GUI_DJI(results, lat, lon, yaw, detection, height, width, meter_per_pixel, cvfont, cvtextsize, cvtextbold, show_lines):
    target_positions = []
    
    """GUI when detecting something for DJI drone"""

    for det in detection:
        x1, y1, x2, y2, conf, cls = det
        centerx = int((x1 + x2) / 2)
        centery = int((y1 + y2) / 2)
        ypixel_to_target, xpixel_to_target = height // 2 - centery, centerx - width // 2
        ymeter_to_target, xmeter_to_target = meter_per_pixel * ypixel_to_target, meter_per_pixel * xpixel_to_target

        north_x = int(width // 2 + int(2 / meter_per_pixel) * np.cos(np.radians(yaw + 90)))
        north_y = int(height // 2 - int(2 / meter_per_pixel) * np.sin(np.radians(yaw + 90)))
        dx_north = north_x - width // 2
        dy_north = north_y - height // 2
        dx_target = centerx - width // 2
        dy_target = centery - height // 2

        """Calculate the angle using arctan2"""
        angle_radians = np.arctan2(dy_target * dx_north - dy_north * dx_target, dx_target * dx_north + dy_target * dy_north)
        angle_degrees = np.degrees(angle_radians)
        angle_degrees = (angle_degrees + 360) % 360
        


        target_point_latitude, target_point_longitude = calculate_target_position(
            lat,
            lon,
            abs(xmeter_to_target),
            abs(ymeter_to_target),
            angle_degrees
        )

        target_positions.append((target_point_latitude, target_point_longitude))

        if show_lines:
            # Draw north direction indecator (2 meters)
            
            cv2.line(results, (width // 2, height // 2), (north_x, north_y), (255, 255, 0), 2)
           
            # Draw lines from center to target
            cv2.line(results, (centerx, centery), (width // 2, height // 2), (0, 255, 0), 2)
            cv2.line(results, (centerx, centery), (centerx, height // 2), (255, 0, 0), 2)
            cv2.line(results, (centerx, height // 2), (width // 2, height // 2), (0, 0, 255), 2)

            # Put text for pixel distance
            cv2.putText(results,
                        str(xpixel_to_target),
                        ((centerx + width // 2) // 2, 
                        height // 2 - 5),
                        cvfont, cvtextsize, 
                        (0, 0, 255), 
                        cvtextbold, 
                        cv2.LINE_AA)
            
            cv2.putText(results, 
                        str(ypixel_to_target), 
                        (centerx + 5, 
                        (centery + height // 2) // 2), 
                        cvfont, 
                        cvtextsize, 
                        (255, 0, 0), 
                        cvtextbold, 
                        cv2.LINE_AA)

            # Put text for meter distance
            cv2.putText(results, 
                        str(np.round(xmeter_to_target, 3)), 
                        ((centerx + width // 2) // 2, 
                        height // 2 + 40), 
                        cvfont, 
                        cvtextsize, 
                        (0, 0, 255), 
                        cvtextbold, 
                        cv2.LINE_AA)
            
            cv2.putText(results, 
                        str(np.round(ymeter_to_target, 3)), 
                        (centerx - 100, 
                        (centery + height // 2) // 2), 
                        cvfont, cvtextsize, 
                        (255, 0, 0), 
                        cvtextbold, 
                        cv2.LINE_AA)


    return target_positions

def deleted_ID_GUI(delete_ID_all, garbage_database_raw_all, lat,lon,heading, m_per_p, width, height, results,radius):
    
    """Processes tracking data to calculate and plot positions of deleted tracked items"""

    deleted_ID = {}
    for track_id in delete_ID_all:
        if track_id in garbage_database_raw_all:
            
            deleted_ID = {
                    int(track_id): {
                        'lat': garbage_database_raw_all[track_id]["positions"][0][0],
                        'lon': garbage_database_raw_all[track_id]["positions"][0][1]
                    }
                    for track_id in delete_ID_all if track_id in garbage_database_raw_all
            }
            
            position = garbage_database_raw_all[track_id]["positions"][0]
            target_lat, target_lon = position[0], position[1]

            """Calculate distance using haversine"""

            distance_m = haversine(
                lat,
                lon,
                target_lat,
                target_lon
            )
            distance_px = distance_m / m_per_p

            """Calculte bearing"""

            absolute_bearing_deg = bearing(
                lat,
                lon,
                target_lat,
                target_lon
            )

            """Adjust bearing based on the drone's yaw"""

            drone_yaw = heading 
            relative_bearing_deg = (absolute_bearing_deg - drone_yaw + 360) % 360
            bearing_rad = np.radians(relative_bearing_deg)

            """Calculate target coordinates based on relative bearing"""

            center_x = width // 2
            center_y = height // 2
            target_x = int(center_x + distance_px * np.sin(bearing_rad))
            target_y = int(center_y - distance_px * np.cos(bearing_rad))

            """Draw circle"""

            radius_px = int(radius / m_per_p)
            cv2.circle(results, (target_x, target_y), radius_px, (0, 140, 255), 3)  

    return deleted_ID