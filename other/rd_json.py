import json
import cv2

with open('frame_5460.json') as json_file:
    data = json.load(json_file)
    
    p = data['shapes']

    #print(len(p))

    l = p[0]

    line = l['label']
    points = l['points']

    print(line)
    print(len(points))




    for p in points:
    	print(p)

    '''
    for p in data['people']:
        print('Name: ' + p['name'])
        print('Website: ' + p['website'])
        print('From: ' + p['from'])
        print('')
    '''
