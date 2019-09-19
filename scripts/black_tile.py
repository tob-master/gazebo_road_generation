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
    
    parser.add_argument("-width","--tile_width",  help="set tile width in cm", type=int)
    parser.add_argument("-height","--tile_height", help="set tile height in cm",type=int)
    args = parser.parse_args()
    
    return args.tile_width, args.tile_height


def main():

    parser = argparse.ArgumentParser()
    
    TILE_WIDTH, TILE_HEIGHT = get_args(parser)

       
    IMAGE_WIDTH  = int(cm2pixel(TILE_WIDTH))
    IMAGE_HEIGHT = int(cm2pixel(TILE_HEIGHT)) 
     
    print(IMAGE_WIDTH)
    print(IMAGE_HEIGHT)
   
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)
    
    context = cairo.Context(surface)
    
    context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    context.set_source_rgba(0, 0, 0, 255)
    context.fill()
       
    scenario_name = "black_tile_w" + str(TILE_WIDTH) + "_h" + str(TILE_HEIGHT) + ".png"    
    
    print("saved: ", scenario_name[:-4])
    surface.write_to_png(scenario_name)


if __name__ == "__main__":
    main()
