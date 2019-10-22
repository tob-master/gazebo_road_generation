

import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5

def cm2pixel(cm):
    return PIXEL_PER_CM * cm 


def get_road_length(parser):
	parser.add_argument("-type","--cross_type", help="set cross type",type=str)
	args = parser.parse_args()
	return args.cross_type

def main():

    parser = argparse.ArgumentParser()
    CROSS_TYPE = get_road_length(parser)


    LINE_WIDTH  = 4
    LINE_LENGTH = 55
	
    IMAGE_WIDTH = int(LINE_LENGTH * PIXEL_PER_CM)
    IMAGE_HEIGHT= int(LINE_WIDTH * PIXEL_PER_CM)
    
    dash_size     = cm2pixel(9)
    dash_gap_size = cm2pixel(6)
    dash_offset   = cm2pixel(0)
    give_way_line_size = cm2pixel(LINE_WIDTH)

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    


    context = cairo.Context(surface)


    context.set_source_rgb(255, 255, 255)
    #context.paint()
    context.set_line_width(give_way_line_size)

	#x_mid = IMAGE_WIDTH / 2
	#y_mid = HEIGHT / 2

	 


    context.set_dash([dash_size, dash_gap_size],dash_offset)
    context.move_to(0,IMAGE_HEIGHT/2)
    context.line_to(IMAGE_WIDTH,IMAGE_HEIGHT/2)
    
    context.stroke()


    surface.write_to_png('give_way_line.png')

if __name__ == "__main__":
    main()





