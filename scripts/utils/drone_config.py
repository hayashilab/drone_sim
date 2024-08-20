connection_string = 'udp:0.0.0.0:14550'
model_path = 'weight/best_newdata.pt'
time_interval = 1.00
cvtextsize = 1
cvtextbold = 2
cvfont = 'FONT_HERSHEY_SIMPLEX' 
camera_matrix = [
    [822.476944, 0.000000, 646.825246],
    [0.000000, 823.343500, 376.344771],
    [0.000000, 0.000000, 1.000000]
]
mission_file = 'missions/kyutech_mission.txt'  # 'missions/hokuto_mission.txt' | 'missions/kyutech_mission.txt' 
topic_image_raw = "/webcam/image_raw"
topic_sonar = "/sonar"
classNames = ["garbage"]
detectClassNames = ["garbage"]