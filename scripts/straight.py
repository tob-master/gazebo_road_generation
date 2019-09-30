#!/usr/bin/python

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
    parser.add_argument("-l","--road_length", help="set road length in cm",type=int)
    parser.add_argument("-o","--dash_offset", help="set dash offset in cm",type=int)

    args = parser.parse_args()
    return args.road_length, args.dash_offset



def draw_line(context,line_width,road_witdh,road_length,x_mid,mid_line_lenght,mid_line_gap,DASH_OFFSET):
    
    context.set_line_width(line_width)
    context.move_to(x_mid , 0)
    context.set_dash([mid_line_lenght, mid_line_gap],DASH_OFFSET)
    context.line_to(x_mid, road_length)
    context.stroke()

    context.set_dash([mid_line_lenght, 0],0)
    context.move_to(x_mid + road_witdh , 0)
    context.line_to(x_mid + road_witdh, road_length)
    context.stroke()

    context.set_dash([mid_line_lenght, 0],0)
    context.move_to(x_mid - road_witdh , 0)
    context.line_to(x_mid - road_witdh, road_length)
    context.stroke()

def main():

    parser = argparse.ArgumentParser()
    ROAD_LENGTH, DASH_OFFSET = get_args(parser)
    
    IMAGE_WIDTH = 100 * PIXEL_PER_CM
    IMAGE_HEIGHT= ROAD_LENGTH * PIXEL_PER_CM
    
    print("image_height",IMAGE_HEIGHT)
    print("image_width",IMAGE_WIDTH)

    line_width             = cm2pixel(2)
    road_witdh             = cm2pixel(45)
    mid_line_lenght        = cm2pixel(20)
    mid_line_gap           = cm2pixel(20)
    road_length            = cm2pixel(ROAD_LENGTH)
    DASH_OFFSET_            = cm2pixel(DASH_OFFSET)
    x_mid = IMAGE_WIDTH / 2

    print("line_widht",line_width)
    print("road_width",road_witdh)
    print("mid_line_length",mid_line_lenght)
    print("road_length",road_length)

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)

    context = cairo.Context(surface)
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 0)
    context.fill()
    
    context.set_source_rgba(255, 255, 255, 255)

    draw_line(context,line_width,road_witdh,road_length,x_mid,mid_line_lenght,mid_line_gap,DASH_OFFSET_)

    scenario_name = 'straight_' + str(ROAD_LENGTH) + '_o_' + str(DASH_OFFSET) + '.png'
    print("saved: ", scenario_name[:-4])
    surface.write_to_png(scenario_name)

if __name__ == "__main__":
    main()




