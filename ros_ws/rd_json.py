import json

with open('img_0000.json') as json_file:
    data = json.load(json_file)
    
    p = data['shapes']

    #print(len(p))

    l = p[0]

    line = l['label']
    points = l['points']

    print(line)
    print(len(points))




    for p in points:
    	print(type(p[0]))

    '''
    for p in data['people']:
        print('Name: ' + p['name'])
        print('Website: ' + p['website'])
        print('From: ' + p['from'])
        print('')
    '''
