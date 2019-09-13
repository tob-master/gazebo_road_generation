import cairocffi as cairo
import math

WIDTH = 500
HEIGHT = 500
PIXEL_SCALE = 1

surface = cairo.ImageSurface(cairo.FORMAT_RGB24,
                             WIDTH*PIXEL_SCALE,
                             HEIGHT*PIXEL_SCALE)

x_offset = 0
y_offset = 0

context = cairo.Context(surface)
context.scale(PIXEL_SCALE, PIXEL_SCALE)


context.set_source_rgb(0, 0, 0)


context.move_to(50 , 0)
context.line_to(10 , 41)
context.stroke()




surface.write_to_png('black_tile.png')
