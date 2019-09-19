#!/usr/bin/python

import cairocffi as cairo
import math
import argparse


'''

Gefolgt von einem Park-Schild koennen parallele Parkmoeglichkeiten in den
naechsten 20m vorkommen.
Mindestens eine parallele Parkmoeglichkeit ist vorhanden.
Weisse Boxen koennen 20 - 200 mm von der rechten Fahrspur entfernt stehen.
Boxen sind mindestens 100 * 100 mm.
Parken Verboten Zonen koennen vorkommen (Manoevrieren erlaubt).
Anfang und Ende der Parkzone ist durch Box oder nicht parken Markierung gekennzeichent.
Von der Startlinie an werden die Parkmoeglichkeiten immer groesser.
Letzter moeglicher Parkbereich ist mindestens 700 mm.
Kleinere von unter 400mm sind ueberall verteilt.

Die Einfahrt und Ausfaehrt hat einen Winkel von 45 bis 60.
Die Breite der Parkzone ist 300mm.
'''



PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5

def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 

#def meter2pixel(m):
#	return (PIXEL_PER_METER) * m 


def get_args(parser):
    parser.add_argument("-l","--parking_lot_length", help="set parking lot length in cm",type=int)
    parser.add_argument("-a","--parking_lot_angle", help="set angle of parking lot entry and exit",type=int)
    parser.add_argument("-p","--line_padding", help="set line padding",type=int)

    args = parser.parse_args()
    return args.parking_lot_length, args.parking_lot_angle, args.line_padding



def draw_parallel_parking_lot(context,line_width,road_width,parking_length,x_mid,PARKING_LOT_ANGLE,LINE_PADDING):
    
    context.set_line_width(line_width)
    
    context.move_to(x_mid, 0)
    
    
    print(PARKING_LOT_ANGLE)
    print( math.tan(PARKING_LOT_ANGLE * math.pi/180.0))
    
    y_diff = road_width / math.tan(PARKING_LOT_ANGLE * math.pi / 180.0)
    
    print("y_diff",y_diff)
    
    context.line_to(x_mid + road_width, y_diff)
    context.stroke()
    
    context.move_to(x_mid, parking_length)
    context.line_to(x_mid + road_width, parking_length - y_diff)
    context.stroke()
    

    context.move_to(x_mid + road_width, parking_length - y_diff + LINE_PADDING)
    context.line_to(x_mid + road_width, y_diff - LINE_PADDING)
    context.stroke()
    
    #context.set_dash([mid_line_lenght, 0],0)
    #context.move_to(x_mid - road_witdh , 0)
    #context.line_to(x_mid - road_witdh, road_length)
    #context.stroke()

def main():

    parser = argparse.ArgumentParser()
    PARKING_LOT_LENGTH, PARKING_LOT_ANGLE, LINE_PADDING = get_args(parser)
    
    IMAGE_WIDTH = 100 * PIXEL_PER_CM
    IMAGE_HEIGHT= PARKING_LOT_LENGTH * PIXEL_PER_CM
    
    print("image_height",IMAGE_HEIGHT)
    print("image_width",IMAGE_WIDTH)

    line_width             = cm2pixel(2)
    road_width             = cm2pixel(30)
    parking_length         = cm2pixel(PARKING_LOT_LENGTH)
    x_mid                  = IMAGE_WIDTH / 2

    print("line_widht",line_width)
    print("road_width",road_width)
    print("parking_length",parking_length)

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)

    context = cairo.Context(surface)
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 0)
    context.fill()
    
    context.set_source_rgba(255, 255, 255, 255)

    draw_parallel_parking_lot(context,line_width,road_width,parking_length,x_mid,PARKING_LOT_ANGLE,LINE_PADDING)

    scenario_name = 'parallel_l' + str(PARKING_LOT_LENGTH) + '_a' + str(PARKING_LOT_ANGLE) + '.png'
    surface.write_to_png(scenario_name)

if __name__ == "__main__":
    main()




