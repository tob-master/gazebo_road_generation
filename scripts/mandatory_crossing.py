

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


    ROAD_WIDTH  = 1
    ROAD_LENGTH = 1
	

    IMAGE_WIDTH = ROAD_WIDTH * PIXEL_PER_METER
    IMAGE_HEIGHT= ROAD_LENGTH * PIXEL_PER_METER



    normal_road_line_width = cm2pixel(2)
    road_witdh             = cm2pixel(45)
    mid_line_lenght        = cm2pixel(20)
    mid_line_gap           = cm2pixel(20)
    road_length            = cm2pixel(ROAD_LENGTH)

    offset = 0

    print(normal_road_line_width)
    print(road_witdh)
    print(mid_line_lenght)
    print(mid_line_gap)
    print(road_length)

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    




		




    context = cairo.Context(surface)


    context.set_source_rgb(255, 255, 255)
    context.set_line_width(10)

    x_mid = IMAGE_WIDTH / 2
	#y_mid = HEIGHT / 2

	 


    context.set_dash([mid_line_lenght, mid_line_gap],offset)
    context.arc(0, 500, 475, -math.pi/2,0 )
    context.stroke()
	
    context.arc(0, 500, 250, -math.pi/2,0)
    context.stroke()


    surface.write_to_png('mandatory_crossing.png')

if __name__ == "__main__":
    main()




