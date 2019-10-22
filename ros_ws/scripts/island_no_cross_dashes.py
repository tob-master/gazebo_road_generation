#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 19 15:38:41 2019

@author: tb
"""

import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5





def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 


road_width = cm2pixel(45)
line_width = cm2pixel(2)

IMAGE_WIDTH  = road_width
IMAGE_HEIGHT = line_width

surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
context = cairo.Context(surface)

context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
context.set_source_rgba(0, 0, 0, 0)
context.fill()

context.set_source_rgba(255, 255, 255, 255)


drawing_distance = 0
context.set_line_width(line_width)

marking_width = cm2pixel(4.5)


while drawing_distance < IMAGE_WIDTH:
    
    

    context.move_to(drawing_distance, line_width/2)
    context.line_to(drawing_distance+marking_width,line_width/2)
    context.stroke()
             
    drawing_distance += marking_width * 2
    
    
surface.write_to_png('island_no_cross_walk_dashes.png')