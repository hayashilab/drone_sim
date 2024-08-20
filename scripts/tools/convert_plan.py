import json

with open('Hokuto.json') as file:
    data = json.load(file)

mission_items = data['mission']['items']

with open('output.txt', 'w') as file:
    file.write('QGC WPL 110\n')
    for i, item in enumerate(mission_items):
        if 'params' in item:
            params  = item['params']
            command = item['command']
            frame   = item['frame']
            lat     = params[4]
            lon     = params[5]
            alt     = params[6]
        else:
            command = 0
            frame   = 0
            lat     = 0.0
            lon     = 0.0
            alt     = 0.0

        file.write(f"{i} 0 {frame} {command} 0.000000 0.000000 0.000000 0.000000 {lat} {lon} {alt} 1\n")