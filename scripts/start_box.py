import cairocffi as cairo
import math

WIDTH = 500
HEIGHT= 750
PIXEL_SCALE = 1

surface = cairo.ImageSurface(cairo.FORMAT_RGB24,
                             WIDTH*PIXEL_SCALE,
                             HEIGHT*PIXEL_SCALE)


context = cairo.Context(surface)
context.scale(PIXEL_SCALE, PIXEL_SCALE)

context.set_source_rgb(255, 255, 255)


x_mid = WIDTH / 2
#y_mid = HEIGHT / 2

 




context.set_line_width(10)



context.set_dash([100, 0],0)
context.move_to(x_mid + 112.5 , 0)
context.line_to(x_mid + 112.5, 750)
context.stroke()

context.set_dash([100, 0],0)
context.move_to(x_mid - 112.5 , 0)
context.line_to(x_mid - 112.5, 750)
context.stroke()




surface.write_to_png('start_box.png')
