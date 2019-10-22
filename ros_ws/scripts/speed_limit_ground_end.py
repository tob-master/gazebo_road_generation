#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 14:47:24 2019

@author: tb
"""
import cairo

if __name__ == '__main__':
    

    PIXEL_PER_METER = 500
    PIXEL_PER_CM    = 5

    def cm2pixel(cm):
        
        return PIXEL_PER_CM * cm 


    x_width = cm2pixel(20)
    x_height = cm2pixel(50)
    line_width = cm2pixel(2)
    
    IMAGE_WIDTH  = x_width
    IMAGE_HEIGHT = x_height
    

    
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
    ctx = cairo.Context(surface)
    ctx.set_source_rgba(255, 255, 255, 255)
    ctx.set_line_width(line_width)


    ctx.move_to(5,5)
    ctx.line_to(IMAGE_WIDTH-5,IMAGE_HEIGHT-5)
    ctx.stroke()

    ctx.move_to(IMAGE_WIDTH-5,5)
    ctx.line_to(5,IMAGE_HEIGHT-5)
    ctx.stroke()


    del ctx

    name = "speed_limit_ground_end.png"

    print(name)

    surface.write_to_png(name)