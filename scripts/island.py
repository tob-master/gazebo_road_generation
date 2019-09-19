#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 17 19:26:18 2019

@author: tb
"""


import cairocffi as cairo
import math
import argparse


PIXEL_PER_METER = 500
PIXEL_PER_CM    = 5


IMAGE_WIDTH = 1000
IMAGE_HEIGHT = 500


def cm2pixel(cm):
    
    return PIXEL_PER_CM * cm 




surface = cairo.ImageSurface(cairo.FORMAT_ARGB32,IMAGE_WIDTH,IMAGE_HEIGHT)    
context = cairo.Context(surface)

context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
context.set_source_rgba(0, 0, 0, 0)
context.fill()
context.set_source_rgba(255, 255, 255, 255)


line_width             = cm2pixel(2)
road_witdh             = cm2pixel(45)

island_width           = cm2pixel(30)
island_height          = cm2pixel(90)

print(island_width)


x_mid = IMAGE_WIDTH / 2

context.set_line_width(line_width)



'''
context.move_to(x_mid, 0)
context.curve_to(x_mid , 100, x_mid+20, 150, x_mid + 10, 200)
context.curve_to(x_mid+50, 220, x_mid+island_width/2, 300, x_mid+island_width/2, island_height)
context.stroke()

context.move_to(x_mid+road_witdh, 0)
context.curve_to(x_mid+ road_witdh, 100, x_mid+20+ road_witdh, 150, x_mid + 10+ road_witdh, 200)
context.curve_to(x_mid+50+ road_witdh, 250, x_mid+ road_witdh, 300, x_mid+island_width/2+ road_witdh, island_height)
context.stroke()
'''




context.move_to(x_mid-road_witdh, 0)
context.curve_to(x_mid -5 -road_witdh, 75, x_mid - 20 - road_witdh, 250, x_mid - 45 - road_witdh, 310)
context.curve_to(x_mid-island_width/2-road_witdh, 400, x_mid-island_width/2-road_witdh, 450, x_mid-island_width/2-road_witdh, island_height)
context.stroke()

context.move_to(x_mid, 0)
context.curve_to(x_mid -5, 75, x_mid - 20, 250, x_mid - 45, 310)
context.curve_to(x_mid-island_width/2, 400, x_mid-island_width/2, 450, x_mid-island_width/2, island_height)
context.stroke()

context.move_to(x_mid, 0)
context.curve_to(x_mid + 5,75, x_mid + 20, 250, x_mid + 45, 310)
context.curve_to(x_mid+island_width/2, 400, x_mid+island_width/2, 450, x_mid+island_width/2, island_height)
context.stroke()


context.move_to(x_mid + road_witdh, 0)
context.curve_to(x_mid + 5 + road_witdh, 75, x_mid + 20 + road_witdh, 250, x_mid + 45 + road_witdh, 310)
context.curve_to(x_mid+island_width/2+ road_witdh, 400, x_mid+island_width/2+ road_witdh, 450, x_mid+island_width/2+ road_witdh, island_height)
context.stroke()





context.move_to(x_mid - island_width/2 - line_width/2, island_height)
context.line_to(x_mid + island_width/2 + line_width/2, island_height)
context.stroke()





crosswalk_marking_width  = cm2pixel(4)
context.set_line_width(crosswalk_marking_width)


#context.set_line_cap(cairo.LINE_CAP_ROUND)

y_offset = cm2pixel(((20.0 / math.cos(27.0 * math.pi / 180.0)) * 2 + 150)/10)

print(y_offset)




h = 90
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h

context.move_to(x_mid   , island_height    )
context.line_to(x_mid + a, island_height - g)
context.stroke()


h = 60
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h
context.move_to(x_mid   , island_height - y_offset    )
context.line_to(x_mid + a, island_height - g -y_offset)
context.stroke()


h = 100
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h
context.move_to(x_mid   , island_height - y_offset    )
context.line_to(x_mid - a, island_height + g -y_offset)
context.stroke()



h = 35
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h
context.move_to(x_mid   , island_height - y_offset*2    )
context.line_to(x_mid + a, island_height - g -y_offset*2)
context.stroke()

h = 50
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h
context.move_to(x_mid   , island_height - y_offset*2    )
context.line_to(x_mid - a, island_height + g -y_offset*2)
context.stroke()


h = 15
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h
context.move_to(x_mid   , island_height - y_offset*3    )
context.line_to(x_mid + a, island_height - g -y_offset*3)
context.stroke()

h = 20
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h
context.move_to(x_mid   , island_height - y_offset*3    )
context.line_to(x_mid - a, island_height + g -y_offset*3)
context.stroke()


context.rectangle(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
context.set_source_rgba(0, 0, 0, 0)
context.fill()
context.stroke()



'''
h = 30
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h



context.move_to(x_mid + 40    , island_height    )
context.line_to(x_mid + 40 + a, island_height - g)
context.stroke()


h = 80
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h

context.move_to(x_mid -10    , island_height    )
context.line_to(x_mid -10 + a, island_height - g)
context.stroke()


h = 200
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h


context.move_to(x_mid -10    , island_height     - cm2pixel(15))
context.line_to(x_mid -10 + a, island_height - g -cm2pixel(15))
context.stroke()


h = 400
a = math.cos(53.0 * math.pi / 180.0) * h
g = math.sin(53.0 * math.pi / 180.0) * h

context.move_to(x_mid -150    , island_height    -cm2pixel(15))
context.line_to(x_mid -150 + a, island_height - g-cm2pixel(15))
context.stroke()



black_x = 150 * math.tan( 27.0 * math.pi / 180.0)
print(black_x)

white_y = 40 / math.tan( 27.0 * math.pi / 180.0)
print(cm2pixel(15))


#tan = g/a
'''

'''
g = cm2pixel(15)


gap_offset  =  g / math.tan(53.0 * math.pi / 180.0)




marking_horizontal_width =  40  / math.cos(27.0 * math.pi / 180.0)

"""cm2pixel(4) cm2pixel(15)"""

context.move_to(x_mid+20-marking_horizontal_width, island_height)
context.line_to(x_mid+20+a, island_height-g)
context.stroke()

print(gap_offset)


context.move_to(x_mid-road_witdh, 0)
context.curve_to(x_mid - 10- road_witdh, 100, x_mid-30- road_witdh, 150, x_mid - 40- road_witdh, 200)
context.curve_to(x_mid-50- road_witdh, 250, x_mid-island_width/2- road_witdh, 300, x_mid-island_width/2- road_witdh, island_height)
context.stroke()


context.move_to(x_mid-island_width/2, island_height)
context.line_to(x_mid+island_width/2, island_height)
'''



surface.write_to_png('island.png')