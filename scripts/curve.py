import cairocffi as cairo
import math

WIDTH = 5000
HEIGHT = 5000
PIXEL_SCALE = 1

surface = cairo.ImageSurface(cairo.FORMAT_RGB24,
                             WIDTH*PIXEL_SCALE,
                             HEIGHT*PIXEL_SCALE)

x_offset = 0
y_offset = 0

context = cairo.Context(surface)
context.scale(PIXEL_SCALE, PIXEL_SCALE)

context.set_source_rgb(255, 255, 255)


x_mid = WIDTH / 2
#y_mid = HEIGHT / 2
y_mid = 0
 

context.set_line_width(10)

def draw_curve(ctx, mid_point_x, mid_point_y, radius, start_angle, end_angle, rotation, track_width, dash_length, dash_offset):


	start_angle_rad = (start_angle / 180.0) * math.pi
	end_angle_rad   = (end_angle   / 180.0) * math.pi

	if(rotation == 'pos'):
		ctx.set_dash([dash_length, dash_length],dash_offset)
		ctx.arc(mid_point_x, mid_point_y, radius, start_angle_rad, end_angle_rad)

		
		#end_point_x = 
		#end_point_y = 


		ctx.stroke()

		ctx.set_dash([dash_length, 0])

		ctx.arc(mid_point_x, mid_point_y, radius + track_width, start_angle_rad, end_angle_rad)
		ctx.stroke()

		ctx.arc(mid_point_x, mid_point_y, radius - track_width, start_angle_rad, end_angle_rad)
		ctx.stroke()

	if(rotation == 'neg'):
		ctx.set_dash([dash_length, dash_length],dash_offset)
		ctx.arc_negative(mid_point_x, mid_point_y, radius, start_angle_rad, end_angle_rad)
		ctx.stroke()

		ctx.set_dash([dash_length, 0])

		ctx.arc_negative(mid_point_x, mid_point_y, radius + track_width, start_angle_rad, end_angle_rad)
		ctx.stroke()

		ctx.arc_negative(mid_point_x, mid_point_y, radius - track_width, start_angle_rad, end_angle_rad)
		ctx.stroke()







draw_curve(context, 5000, 0, 2500, -180, -90, 'neg', 225, 1000,0)


#draw_curve(context, x_mid+320, y_mid, 160, 180, 70, 'neg', 40, 20, 0)


#draw_line()


'''
context.set_dash([20, 20],18)

context.move_to(mv_pos_x , mv_pos_y)

a = math.cos(20*math.pi / 180) * 300 
g = math.sin(20*math.pi / 180) * 300


mv_pos_x = mv_pos_x - a
mv_pos_y = mv_pos_y - g

context.line_to(mv_pos_x, mv_pos_y)
context.stroke()

'''
#context.set_line_width(2)
surface.write_to_png('curve.png')
