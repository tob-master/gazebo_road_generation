import cairocffi as cairo
import math

WIDTH = 500
HEIGHT= 750
PIXEL_SCALE = 1

surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,
                             WIDTH*PIXEL_SCALE,
                             HEIGHT*PIXEL_SCALE)


context = cairo.Context(surface)
context.scale(PIXEL_SCALE, PIXEL_SCALE)

context.set_source_rgba(0.0, 0.0, 0.0, 0.0) # transparent black
context.rectangle(0, 0, 512, 128)
context.fill()


# set writing color to white
context.set_source_rgb(1, 1, 1)

# write text
context.move_to(100,50)
context.show_text("hello")

# commit to surface
context.stroke()




surface.write_to_png('transparent.png')



