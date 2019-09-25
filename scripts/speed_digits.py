#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 19 17:41:27 2019

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

IMAGE_WIDTH  = 200
IMAGE_HEIGHT = 200

surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
context = cairo.Context(surface)

context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
context.set_source_rgba(0, 0, 0, 0)
context.fill()

context.set_source_rgba(255, 255, 255, 255)

#context.select_font_face("Linear-Antiqua", cairo.FONT_SLANT_NORMAL, 
#cairo.FONT_WEIGHT_NORMAL)

context.select_font_face("Kinnari", cairo.FONT_SLANT_NORMAL, 
cairo.FONT_WEIGHT_NORMAL)


context.set_font_size(70)

context.move_to(50, 50)
context.show_text("31")

surface.write_to_png('digit.png')