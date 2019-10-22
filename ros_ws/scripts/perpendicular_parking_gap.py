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



def main():

    
    IMAGE_WIDTH = 100 * PIXEL_PER_CM

    print("image_width",IMAGE_WIDTH)

    line_width             = cm2pixel(2)
    gap_length             = cm2pixel(50)
    gap_width              = cm2pixel(35)
    x_mid                  = IMAGE_WIDTH / 2



    
    IMAGE_HEIGHT= int(gap_length)
    
    print("image_height",IMAGE_HEIGHT)

    print("line_widht",line_width)


    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)

    context = cairo.Context(surface)
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 0)
    context.fill()
    
    context.set_source_rgba(255, 255, 255, 255)
    context.set_line_width(line_width)
    context.move_to(x_mid - gap_width/2, 0)
    context.line_to(x_mid - gap_width/2, gap_length)
    context.stroke()    
    
    context.move_to(x_mid + gap_width/2, 0)    
    context.line_to(x_mid + gap_width/2, gap_length)
    context.stroke()       

    scenario_name = 'perpendicular_gap.png'
    surface.write_to_png(scenario_name)

if __name__ == "__main__":
    main()




