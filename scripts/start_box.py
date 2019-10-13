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
    parser.add_argument("-rll","--right_lane_length", help="set right lane length in cm",type=int)
    parser.add_argument("-lll","--left_lane_length", help="set left lane length in cm",type=int)   
    args = parser.parse_args()
    return args.right_lane_length, args.left_lane_length


def draw_start_box(context, right_length, left_length, x_mid, line_width, road_width):
    
    context.set_line_width(line_width)
    
    
    context.move_to(x_mid + road_width , 0)
    context.line_to(x_mid + road_width, right_length)
    context.stroke()

    context.move_to(x_mid - road_width , right_length)
    context.line_to(x_mid - road_width, right_length - left_length)
    context.stroke()
    
    
    context.move_to(x_mid - road_width, right_length - line_width/2)
    context.line_to(x_mid + road_width, right_length - line_width/2)
    context.stroke()

def main():

    parser = argparse.ArgumentParser()
    RIGHT_LANE_LENGTH, LEFT_LANE_LENGTH = get_args(parser)
    
    IMAGE_WIDTH = 100 * PIXEL_PER_CM
    IMAGE_HEIGHT= RIGHT_LANE_LENGTH * PIXEL_PER_CM

    line_width             = cm2pixel(2)
    road_width             = cm2pixel(20)
    right_length           = cm2pixel(RIGHT_LANE_LENGTH)
    left_length            = cm2pixel(LEFT_LANE_LENGTH)
    x_mid                  = IMAGE_WIDTH / 2

    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)

    context = cairo.Context(surface)
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 0)
    context.fill()
    
    context.set_source_rgba(255, 255, 255, 255)

    draw_start_box(context, right_length, left_length, x_mid, line_width, road_width)
   

    scenario_name = 'start_box_ll' + str(LEFT_LANE_LENGTH) + '_rl' + str(RIGHT_LANE_LENGTH) + '.png'

    surface.write_to_png(scenario_name)

if __name__ == "__main__":
    main()