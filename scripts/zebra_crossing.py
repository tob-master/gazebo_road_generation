#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Created on Tue Sep 17 16:56:58 2019

@author: tb
"""
"""
In a suburban area, one or more crosswalks may be present. These are marked with several 36 mm to 40 mm wide and 400 mm long 
white markings parallel to the direction of travelwhich are spaced 40 mm apart (cf. Section A.1.6). A crosswalk is indicated 
by a correspondingtraffic sign (cf. Section A.2). On the roadside at each crosswalk “pedestrians” may wait to crossthe road.  
For this purpose two areas are defined which may contain relevant pedestrians.A “pedestrian” is depicted by a small white 
cardboard box in analogy to the static obstacles.In addition, each pedestrian is marked with a pictograph, in order to facilitate 
its detection(cf. Section A.3.2).Multiple pedestrians can be located on the right as well as on the left hand side of the crosswalk.
Pedestrians will always be clearly distinguishable from the view of the approaching vehicle.Only if at least one pedestrian is present 
in the defined zones, the vehicle must stop in frontof the crosswalk. Stopping must be performed with the same regulations as at intersections.
Pedestrians start crossing the street only after the vehicle has stopped. If all relevant pedestrianshave crossed in front of the vehicle, 
the vehicle may continue. Driving on before all pedestriansstarted to cross and have cleared the right lane will be penalized as collisions 
with obstacles.
"""


import cairocffi as cairo
import math


PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5

def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 

#def meter2pixel(m):
#	return (PIXEL_PER_METER) * m 




def draw_crosswalk(ctx, crosswalk_marking_width, crosswalk_marking_height ,IMAGE_WIDTH,IMAGE_HEIGHT):

    
    
    drawing_distance = crosswalk_marking_width
    
    while drawing_distance < IMAGE_WIDTH:
        
        ctx.set_line_width(crosswalk_marking_width)
        ctx.move_to(drawing_distance,0)
        ctx.line_to(drawing_distance,IMAGE_HEIGHT)
        ctx.stroke()
        
        drawing_distance += crosswalk_marking_width * 2
        
        
    



def main():

    
    
    IMAGE_WIDTH  = int(cm2pixel(45 * 2))
    IMAGE_HEIGHT = int(cm2pixel(40))
 
     
    print(IMAGE_WIDTH)
    print(IMAGE_HEIGHT)

    
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)
    context = cairo.Context(surface)
    
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 0)
    context.fill()
    
    context.set_source_rgba(255, 255, 255, 255)
    


    crosswalk_marking_width  = cm2pixel(4)
    crosswalk_marking_height = cm2pixel(40)

    print(crosswalk_marking_width)
    print(crosswalk_marking_height)

    
    draw_crosswalk(context, crosswalk_marking_width, crosswalk_marking_height ,IMAGE_WIDTH,IMAGE_HEIGHT)
    
    scenario_name = "crosswalk.png" 
    

    
    surface.write_to_png(scenario_name)


if __name__ == "__main__":
    main()
