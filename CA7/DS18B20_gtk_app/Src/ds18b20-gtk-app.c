#include <gtk/gtk.h>
//#include <math.h>

static void
drawingArea_resize(	GtkDrawingArea* self,
					gint width,
					gint height,
					gpointer user_data)
{
	g_print("DrawingArea resize! x:%d, y:%d\n", width, height);
}

GtkSizeRequestMode
drawingArea_get_request_mode (
  GtkWidget* widget
)
{
	return GTK_SIZE_REQUEST_HEIGHT_FOR_WIDTH;
}
void drawingArea_measure( GtkWidget* widget,
						  GtkOrientation orientation,
						  int for_size,
						  int* minimum,
						  int* natural,
						  int* minimum_baseline,
						  int* natural_baseline)
{
	if (orientation == GTK_ORIENTATION_HORIZONTAL)
	{
	  // Calculate minimum and natural width
		*minimum = gtk_drawing_area_get_content_width(GTK_DRAWING_AREA(widget));
		if(for_size == -1)
		{
			*natural = *minimum;
		}
		else
		{
			*natural = for_size;
		}
	}
	else // VERTICAL
	{
		*minimum = gtk_drawing_area_get_content_height(GTK_DRAWING_AREA(widget));
		if(for_size == -1)
		{
			*natural = *minimum;
		}
		else
		{
			*natural = for_size;
		}
	}

	*minimum_baseline = -1;
	*natural_baseline = -1;
}
static void
change_colors_btn (GtkWidget *widget,
             gpointer   data)
{
  gtk_widget_queue_draw((GtkWidget *)data);

}

gboolean change_colors (gpointer   data)
{
		gtk_widget_queue_draw((GtkWidget *)data);
		return true;
}
void close_window()
{

}
void draw_gauge_func(GtkDrawingArea *area,
        cairo_t        *cr,
        int             width,
        int             height,
        gpointer        data)
{
	float center_x, center_y;
	float bezel_radius;
	float scale_radius, scale_start_x, scale_start_y, scale_end_x, scale_end_y;
	float scale_radians_start, scale_radians_end, scale_radians_range;
	float scale_control_x1, scale_control_y1;
	float scale_control_x2, scale_control_y2;
	float scale_control_radians, scale_control_k;

	float scale_value_range;
	float scale_tick_degrees;

		//Values passed from AnalogGauge Widget
	#define ANALOGGAUGE_SCALE_SEGMENT_RADIANS_MAX (M_PI/2)
	float scale_radius_offset_from_bezel = 100;
	float scale_value_minimum = -40;
	float scale_value_maximum = 125;

	float scale_degrees_start = 360;	//Always start at a larger degree 2PI = 360
	static float scale_degrees_end   = 90;	//Always end at a smaller degree 2PI = 0

	//TODO: Assert start > end?

	if(scale_degrees_start < scale_degrees_end)
	{
		scale_degrees_end = scale_degrees_start;
	}


	//End Values passed from AnalogGauge Widget
//TODO: Pass in a scale struct and allow multiple scales per gauge!
//TODO: Needle
//TODO: Tell-tale pointers to record highest/ lowest?
//TODO: Alert indicators
	static int color_cnt = 0;

	GdkRGBA red;
	GdkRGBA black;
	GdkRGBA green;
	GdkRGBA yellow;
	GdkRGBA blue;
	GdkRGBA purple;
	GdkRGBA *colors[5] = { &red, &green, &blue, &yellow, &purple};
//	GdkRGBA *fill_colors[5] = { &purple, &yellow, &red, &blue, &green};

	gdk_rgba_parse(&red, 	"red");
	gdk_rgba_parse(&black, 	"black");
	gdk_rgba_parse(&green, 	"green");
	gdk_rgba_parse(&yellow, "yellow");
	gdk_rgba_parse(&blue, 	"blue");
	gdk_rgba_parse(&purple, "purple");

	center_x =width/2;
	center_y =height/2;

	bezel_radius = MIN(center_x,center_y)-10;
	scale_radius = bezel_radius-scale_radius_offset_from_bezel;

	//TODO: Value checks, also +1 for 0C?
	scale_value_range   = ABS(scale_value_maximum)+ ABS(scale_value_minimum);
	scale_tick_degrees = scale_value_range/5;	//show every 5 degrees as a tick mark

	scale_radians_start = scale_degrees_start* M_PI/180;
	scale_radians_end = scale_degrees_end * M_PI/180;
	scale_radians_range = scale_radians_start- scale_radians_end;
	//Stroke Gauge Bezel
	cairo_arc (cr, center_x, center_y, bezel_radius, 0, 2 * G_PI);
	gdk_cairo_set_source_rgba(cr, &red);
	cairo_set_line_width(cr,10);
	cairo_stroke(cr);

	cairo_set_line_width(cr,5);
	//Translate snapshot to center to allow rotation of scale segments around circle center
	cairo_translate(cr, center_x, center_y);
//	//Rotate to the scale start degrees
	cairo_rotate(cr, -scale_radians_start);	//-_- snapshot::rotate uses positive clockwise degrees
	do
	{
		if(scale_radians_range > ANALOGGAUGE_SCALE_SEGMENT_RADIANS_MAX)
		{
			scale_control_radians = M_PI/2;
		}
		else
		{
			scale_control_radians = scale_radians_start - scale_radians_end;
		}

		scale_start_x = scale_radius;
		scale_start_y =  0;

		scale_end_x = scale_radius* cos(scale_control_radians);
		scale_end_y = scale_radius* sin(scale_control_radians);

		scale_control_k = 4*tan(scale_control_radians/4)/3;

		scale_control_x1 = scale_start_x;
		scale_control_y1 = scale_start_y 	+ (scale_radius * scale_control_k);
		scale_control_x2 = scale_end_x 		+ (scale_radius * scale_control_k * sin(scale_control_radians));
		scale_control_y2 = scale_end_y 		- (scale_radius * scale_control_k * cos(scale_control_radians));

#if 0

//		g_print("\ndegrees_start:%f degrees_end:%f degrees_range:%f degrees_rotate:%f\n", scale_degrees_start, scale_degrees_end, scale_degrees_range, scale_control_degrees);
//		g_print("radians_start:%f Ctrl_radians:%f",scale_radians_start, scale_control_radians);
//		g_print("\nRadius:%f: K:%f\nSx :%f Sy :%f\nPx1:%f Py1:%f\nPx2:%f Py2:%f\nEx :%f Ey :%f\n",scale_radius,scale_control_k, scale_start_x,scale_start_y, scale_control_x1, scale_control_y1, scale_control_x2, scale_control_y2, scale_end_x, scale_end_y);

		cairo_move_to(cr, 	0, 0);
		cairo_line_to(cr, 	scale_start_x, scale_start_y);
		gdk_cairo_set_source_rgba(cr, &red);
		cairo_stroke(cr);

		cairo_move_to(cr, 	scale_start_x, scale_start_y);
		cairo_line_to(cr, 	scale_control_x1, scale_control_y1);
		cairo_line_to(cr, 0 ,0);
		gdk_cairo_set_source_rgba(cr, &green);
		cairo_stroke(cr);


		cairo_move_to(cr, 	scale_control_x1, scale_control_y1);
		cairo_line_to(cr, 	scale_control_x2, scale_control_y2);
		cairo_line_to(cr, 0 ,0);
		gdk_cairo_set_source_rgba(cr, &blue);
		cairo_stroke(cr);

		cairo_move_to(cr, 	scale_control_x2, scale_control_y2);
		cairo_line_to(cr, 	scale_end_x, scale_end_y);
		cairo_line_to(cr, 0 ,0);
		gdk_cairo_set_source_rgba(cr, &yellow);
		cairo_stroke(cr);

#endif

		cairo_move_to(cr, 	scale_start_x, scale_start_y);
		cairo_curve_to(cr, 	scale_control_x1, scale_control_y1,
									scale_control_x2, scale_control_y2,
									scale_end_x, scale_end_y);

		gdk_cairo_set_source_rgba(cr, colors[color_cnt]);
		cairo_stroke(cr);

		cairo_rotate(cr, scale_control_radians);	//rotate past new segment

		scale_radians_start -=scale_control_radians;
		scale_radians_range -=ANALOGGAUGE_SCALE_SEGMENT_RADIANS_MAX;
		color_cnt++;
		color_cnt%=5;
	} while(scale_radians_range>0);

	scale_degrees_end -=20;
	if(scale_degrees_end < 0)
	{
		scale_degrees_end = 360;
	}
}

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
  GtkWidget *window;
  GtkWidget *button;
  GtkWidget *vbox;
  GtkWidget *drawingArea;
  GtkWidgetClass *drawingAreaClass;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Window");
  gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);

  g_signal_connect (window, "destroy", G_CALLBACK (close_window), NULL);
  vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);

  drawingArea = gtk_drawing_area_new();
  gtk_widget_set_size_request (drawingArea, 200, 200);
  gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(drawingArea), &draw_gauge_func, NULL, NULL);
  drawingAreaClass = GTK_WIDGET_GET_CLASS(drawingArea);
//  drawingAreaClass->snapshot = draw_gauge_snapshot;
  drawingAreaClass->get_request_mode = drawingArea_get_request_mode;
  drawingAreaClass->measure = drawingArea_measure;

  g_signal_connect (drawingArea, "resize", G_CALLBACK (drawingArea_resize), NULL);

  button = gtk_button_new_with_label ("Change Colors!");
  gtk_widget_set_halign (button, GTK_ALIGN_CENTER);
  gtk_widget_set_valign (button, GTK_ALIGN_CENTER);

  g_signal_connect (button, "clicked", G_CALLBACK (change_colors_btn), (void *)drawingArea);

  gtk_box_append(GTK_BOX(vbox), drawingArea);
  gtk_box_append(GTK_BOX(vbox), button);
  gtk_window_set_child (GTK_WINDOW (window), vbox);

  gtk_window_present (GTK_WINDOW (window));
  g_timeout_add(100, (GSourceFunc)change_colors, (gpointer)drawingArea);
}

int
main (int    argc,
      char **argv)
{
  GtkApplication *app;
  int status;

  app = gtk_application_new (NULL, G_APPLICATION_DEFAULT_FLAGS);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  status = g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  return status;
}

