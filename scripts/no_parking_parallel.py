#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 14:47:24 2019

@author: tb
"""
import cairo
import math

if __name__ == '__main__':
    

    PIXEL_PER_METER = 500
    PIXEL_PER_CM    = 5

    def cm2pixel(cm):
        
        return PIXEL_PER_CM * cm 


    x_width =  cm2pixel(30)
    line_width = cm2pixel(2)
    
    
    
    y_diff = x_width * math.tan(60 * math.pi / 180.0)
    
    IMAGE_WIDTH  = x_width
    IMAGE_HEIGHT = int(y_diff+1)


    
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
    ctx = cairo.Context(surface)
    ctx.set_source_rgba(255, 255, 255, 255)
    ctx.set_line_width(line_width)





    ctx.move_to(0,0)
    ctx.line_to(IMAGE_WIDTH,IMAGE_HEIGHT)
    ctx.stroke()

    ctx.move_to(IMAGE_WIDTH,0)
    ctx.line_to(0,IMAGE_HEIGHT)
    ctx.stroke()
    
    ctx.move_to(0,line_width/2)
    ctx.line_to(IMAGE_WIDTH,line_width/2)
    ctx.stroke()
    
    ctx.move_to(0,IMAGE_HEIGHT - line_width/2)
    ctx.line_to(IMAGE_WIDTH,IMAGE_HEIGHT - line_width/2)
    ctx.stroke()



    del ctx

    name = "no_parking_parallel.png"

    print(name)

    surface.write_to_png(name)
    