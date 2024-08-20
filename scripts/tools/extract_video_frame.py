import cv2
import os

def extract_frames(video_path, output_folder, frame_interval=300):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Error: Could not open video.")
        return
    
    frame_count = 0
    saved_frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        if frame_count % frame_interval == 0:
            frame_filename = os.path.join(output_folder, f"frame_{saved_frame_count:04d}.jpg")
            cv2.imwrite(frame_filename, frame)
            saved_frame_count += 1
            print(f"Saved {frame_filename}")
        
        frame_count += 1

    cap.release()
    print("Frame extraction completed.")


video_path = 'output.avi' 
output_folder = 'extracted_frames' 
frame_interval = 50
extract_frames(video_path, output_folder, frame_interval)
