#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 18 14:31:25 2019

@author: tb
"""



import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5


IMAGE_WIDTH = 1000
IMAGE_HEIGHT = 225


def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 




surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
context = cairo.Context(surface)

context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
context.set_source_rgba(0, 0, 0, 0)
context.fill()
context.set_source_rgba(255, 255, 255, 255)


line_width             = cm2pixel(2)
road_width             = cm2pixel(45)
island_width           = cm2pixel(30)
island_middle_zone_height = cm2pixel(45)
crosswalk_marking_width  = cm2pixel(4)
crosswalk_marking_height = cm2pixel(40)

print(island_middle_zone_height)
print(road_width*2+island_width)

x_mid = IMAGE_WIDTH / 2

context.set_line_width(line_width)

context.move_to(x_mid-island_width/2-road_width, 0)
context.line_to(x_mid-island_width/2-road_width, 225)
context.stroke()

context.move_to(x_mid+island_width/2+road_width, 0)
context.line_to(x_mid+island_width/2+road_width, 225)
context.stroke()

crosswalk_start = x_mid-island_width/2-road_width
crosswalk_end   = x_mid+island_width/2+road_width

no_crosswalk_start  = x_mid-island_width/2
no_crosswalk_end    = x_mid+island_width/2

  
context.set_line_width(line_width)    
context.move_to(no_crosswalk_start,0)
context.line_to(no_crosswalk_start,IMAGE_HEIGHT)
context.stroke() 

context.move_to(no_crosswalk_end,0)
context.line_to(no_crosswalk_end,IMAGE_HEIGHT)
context.stroke() 

    
    
surface.write_to_png('island_crosswalk.png')
