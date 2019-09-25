#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 19 16:56:27 2019

@author: tb
"""

import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5





def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 


start_line_width  = cm2pixel(90)
marking_width     = cm2pixel(1.33)
marking_height    = cm2pixel(1.33)
start_line_height = cm2pixel(4)

IMAGE_WIDTH  = start_line_width
IMAGE_HEIGHT = start_line_height

surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
context = cairo.Context(surface)

context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
context.set_source_rgba(0, 0, 0, 0)
context.fill()

context.set_source_rgba(255, 255, 255, 255)


drawing_distance = 0
context.set_line_width(marking_width)



while drawing_distance < IMAGE_WIDTH:
    
    

    context.move_to(drawing_distance, marking_height/2)
    context.line_to(drawing_distance+marking_width,marking_height/2)
    context.stroke()
    
             
    drawing_distance += marking_width * 2
    
drawing_distance = marking_width

while drawing_distance < IMAGE_WIDTH:
    
    

    context.move_to(drawing_distance, (marking_height/2)*3)
    context.line_to(drawing_distance+marking_width,(marking_height/2)*3)
    context.stroke()
    
             
    drawing_distance += marking_width * 2
    
drawing_distance = 0   
while drawing_distance < IMAGE_WIDTH:
    
    

    context.move_to(drawing_distance, (marking_height/2)*5)
    context.line_to(drawing_distance+marking_width,(marking_height/2)*5)
    context.stroke()
    
             
    drawing_distance += marking_width * 2
    
    
surface.write_to_png('start_line.png')