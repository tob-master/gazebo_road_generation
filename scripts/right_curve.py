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

def cm2pixel(cm):
    
    return PIXEL_PER_METER * float(cm / 100.0) 


def get_args(parser):
    
    parser.add_argument("-r","--curve_radius", help="set curve radius in cm",type=int)
    parser.add_argument("-o","--dash_offset", help="set dash offset cm",type=int)

    args = parser.parse_args()
    return args.curve_radius, args.dash_offset


def draw_curve(ctx, mid_point_x, mid_point_y, radius, start_angle, end_angle, track_width, dash_length, dash_offset):

	start_angle_rad = (start_angle / 180.0) * math.pi
	end_angle_rad   = (end_angle   / 180.0) * math.pi

	ctx.set_dash([dash_length, dash_length],dash_offset)
	ctx.arc(mid_point_x, mid_point_y, radius, start_angle_rad, end_angle_rad)
	ctx.stroke()

	ctx.set_dash([dash_length, 0])
	ctx.arc(mid_point_x, mid_point_y, radius + track_width, start_angle_rad, end_angle_rad)
	ctx.stroke()

	ctx.arc(mid_point_x, mid_point_y, radius - track_width, start_angle_rad, end_angle_rad)
	ctx.stroke()



def main():

    parser = argparse.ArgumentParser()
    
    CURVE_RADIUS, DASH_OFFSET = get_args(parser)
    
    print(CURVE_RADIUS)
    
    IMAGE_WIDTH  = int(cm2pixel(CURVE_RADIUS)) * 2
    IMAGE_HEIGHT = int(cm2pixel(CURVE_RADIUS)) * 2
    RADIUS       = int(cm2pixel(CURVE_RADIUS)) 
     
    print(IMAGE_WIDTH)
    print(IMAGE_HEIGHT)
    print(RADIUS)
    
    surface = cairo.ImageSurface(cairo.FORMAT_RGB24,IMAGE_WIDTH,IMAGE_HEIGHT)
    
    context = cairo.Context(surface)
    context.set_source_rgb(255, 255, 255)
    
    
    context.set_line_width(10)
    
    road_witdh             = cm2pixel(45)
    mid_line_lenght        = cm2pixel(20)
    mid_line_offset        = cm2pixel(DASH_OFFSET)
    print(road_witdh)
    
    draw_curve(context, 0, 0, RADIUS, 0, 90, road_witdh, mid_line_lenght,mid_line_offset)
    
    scenario_name = "right_curve_r_" + str(CURVE_RADIUS) + "cm.png" 
    

    
    surface.write_to_png(scenario_name)


if __name__ == "__main__":
    main()