import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500


def cm2pixel(cm):
    
    return PIXEL_PER_METER * float(cm / 100.0) 

#def meter2pixel(m):
#	return (PIXEL_PER_METER) * m 


def get_args(parser):
    parser.add_argument("-l","--left_legth", help="set road length",type=int)
   
    args = parser.parse_args()
    return args.road_length

def main():

    parser = argparse.ArgumentParser()
    ROAD_LENGTH = get_args(parser)
    
    
    ROAD_WIDTH  = 1

    IMAGE_WIDTH = ROAD_WIDTH * PIXEL_PER_METER
    IMAGE_HEIGHT= int((ROAD_LENGTH/100.0) * PIXEL_PER_METER)
    print(IMAGE_HEIGHT)

    normal_road_line_width = cm2pixel(2)
    road_witdh             = cm2pixel(45)
    mid_line_lenght        = cm2pixel(20)
    mid_line_gap           = cm2pixel(20)
    road_length            = cm2pixel(ROAD_LENGTH)


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


    #scenario_name = 'straight_' + str(ROAD_LENGTH) + 'cm.png'

    surface.write_to_png('start_box.png')

if __name__ == "__main__":
    main()