#!/usr/bin/python

import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500



def meter2pixel(m):
	return (PIXEL_PER_METER) * m 


def get_road_length(parser):
	parser.add_argument("-len","--road_length", help="set road length",
		            type=int)
	args = parser.parse_args()
	return args.road_length

def main():

	parser = argparse.ArgumentParser()
	ROAD_LENGTH = get_road_length(parser)


	ROAD_WIDTH  = 1
	

	IMAGE_WIDTH = ROAD_WIDTH * PIXEL_PER_METER
	IMAGE_HEIGHT= ROAD_LENGTH * PIXEL_PER_METER




	normal_road_line_width = meter2pixel(0.02)
	road_witdh             = meter2pixel(0.45)
	mid_line_lenght        = meter2pixel(0.2)
	mid_line_gap           = meter2pixel(0.2)
	road_length            = meter2pixel(ROAD_LENGTH)


	print(normal_road_line_width)

	print(road_witdh)

	print(mid_line_lenght)

	print(mid_line_gap)

	print(road_length)


	surface = cairo.ImageSurface(cairo.FORMAT_RGB24,
			             IMAGE_WIDTH,
			             IMAGE_HEIGHT)




		




	context = cairo.Context(surface)
	context.scale(1, 1)

	context.set_source_rgb(255, 255, 255)


	x_mid = IMAGE_WIDTH / 2
	#y_mid = HEIGHT / 2

	 






	context.set_line_width(normal_road_line_width)
	context.move_to(x_mid , 0)
	context.set_dash([mid_line_lenght, mid_line_gap],0)
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


	scenario_name = 'straight_' + str(ROAD_LENGTH) + 'm.png'

	surface.write_to_png(scenario_name)

if __name__ == "__main__":
    main()




