#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 13 11:12:49 2019

@author: tb
"""
import cairocffi as cairo
import math
import argparse

PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5

def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 

#def meter2pixel(m):
#	return (PIXEL_PER_METER) * m 


def get_args(parser):
    
    parser.add_argument("-r","--curve_radius", help="set curve radius in cm",type=int)
    parser.add_argument("-a","--angle", help="set angle in degree",type=int)
    parser.add_argument("-o","--dash_offset", help="set dash offset cm",type=int)

    args = parser.parse_args()
    return args.curve_radius, args.angle, args.dash_offset


def draw_curve(ctx, mid_point_x, mid_point_y, radius, start_angle, end_angle, line_width, track_width, dash_length, dash_offset):
 
    start_angle_rad = (start_angle / 180.0) * math.pi
    end_angle_rad   = (end_angle   / 180.0) * math.pi
    ctx.set_line_width(line_width)

    ctx.set_dash([dash_length, dash_length],dash_offset)
    ctx.arc_negative(mid_point_x, mid_point_y, radius, start_angle_rad, end_angle_rad)
    ctx.stroke()

    ctx.set_dash([dash_length, 0])
    ctx.arc_negative(mid_point_x, mid_point_y, radius + track_width, start_angle_rad, end_angle_rad)
    ctx.stroke()

    ctx.arc_negative(mid_point_x, mid_point_y, radius - track_width, start_angle_rad, end_angle_rad)
    ctx.stroke()



def main():

    parser = argparse.ArgumentParser()
    
    CURVE_RADIUS, ANGLE, DASH_OFFSET = get_args(parser)
    
    
    if ANGLE < 0 or ANGLE > 90:
        print("Angle must be between 0 and 90 degree!")
        return 0
    
    
    print(CURVE_RADIUS)
    
    IMAGE_WIDTH  = int(cm2pixel(CURVE_RADIUS)) * 2
    IMAGE_HEIGHT = int(cm2pixel(CURVE_RADIUS)) * 2
    RADIUS       = int(cm2pixel(CURVE_RADIUS)) 
     
    print(IMAGE_WIDTH)
    print(IMAGE_HEIGHT)
    print(RADIUS)
    
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)
    
    context = cairo.Context(surface)
    
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 0)
    context.fill()
    
    context.set_source_rgba(255, 255, 255, 255)
    
    line_width             = cm2pixel(2)
    road_witdh             = cm2pixel(45)
    mid_line_lenght        = cm2pixel(20)
    mid_line_offset        = cm2pixel(DASH_OFFSET)
    print(road_witdh)
    
    draw_curve(context, 0, 0, RADIUS, 90, 90-ANGLE, line_width,road_witdh, mid_line_lenght,mid_line_offset)
    
    scenario_name = "left_curve_r" + str(CURVE_RADIUS) + "_a" + str(ANGLE) + ".png" 
    
    surface.write_to_png(scenario_name)


if __name__ == "__main__":
    main()