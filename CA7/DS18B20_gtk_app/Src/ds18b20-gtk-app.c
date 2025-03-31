#include <gtk/gtk.h>
#include <math.h>

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

//Draw the gauge bezel and scale from start to end radians, break up radians larger than M_PI into two segments
void draw_gauge(  GtkWidget* widget,
		  	  	  GtkSnapshot* snapshot)
//void draw_gauge(GtkDrawingArea *area,
//        cairo_t        *cr,
//        int             width,
//        int             height,
//        gpointer        data)
{
	GskPathBuilder *builder;
	graphene_point_t center;
	GskPath *path;
	GskStroke *stroke;
	GdkRGBA red, black, green, yellow, blue, purple;
	float width,height;
	float bezel_radius;
	float scale_radius, scale_start_x, scale_start_y, scale_end_x, scale_end_y;
	float scale_radians_start;	//start at 2PI and end would be 0
	float scale_radians_end;
	float scale_control_x1, scale_control_y1;
	float scale_control_x2, scale_control_y2;
	float scale_control_radians, scale_control_degrees, scale_control_k;
	float scale_degrees_range;
	float scale_value_range;
	float scale_tick_degrees;

	//Values passed from AnalogGauge Widget
#define ANALOGGAUGE_SCALE_SEGMENT_DEGREE_MAX 90
	float scale_radius_offset_from_bezel = 100;
	float scale_value_minimum = -40;
	float scale_value_maximum = 125;

	float scale_degrees_start = 360;	//Always start at a larger degree 2PI = 360
//TODO: DEBUG!!
	static float scale_degrees_end   = 360;	//Always end at a smaller degree 2PI = 0

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

//	GdkRGBA *colors[5] = { &red, &green, &blue, &yellow, &purple};
//	GdkRGBA *fill_colors[5] = { &purple, &yellow, &red, &blue, &green};

	gdk_rgba_parse(&red, 	"red");
	gdk_rgba_parse(&black, 	"black");
	gdk_rgba_parse(&green, 	"green");
	gdk_rgba_parse(&yellow, "yellow");
	gdk_rgba_parse(&blue, 	"blue");
	gdk_rgba_parse(&purple, "purple");

	width = gtk_widget_get_width(widget);
	height = gtk_widget_get_height(widget);
	center.x =width/2;
	center.y =height/2;

	bezel_radius = fmin(center.x,center.y)-10;
	scale_radius = bezel_radius-scale_radius_offset_from_bezel;

	//TODO: Value checks, also +1 for 0C?
	scale_value_range   = ABS(scale_value_maximum)+ ABS(scale_value_minimum);
	scale_tick_degrees = scale_value_range/5;	//show every 5 degrees as a tick mark

	scale_degrees_range = scale_degrees_start - scale_degrees_end;
	scale_radians_start = scale_degrees_start* M_PI/180;
	scale_radians_end = scale_degrees_end * M_PI/180;

	//Stroke Gauge Bezel
	builder = gsk_path_builder_new();
	gsk_path_builder_add_circle(builder, &center, bezel_radius);
	path = gsk_path_builder_free_to_path(builder);
	stroke = gsk_stroke_new(10.0f);
	gtk_snapshot_append_stroke(snapshot, path, stroke, &black);
	gsk_stroke_free(stroke);
	gsk_path_unref(path);

	//Translate snapshot to center to allow rotation of scale segments around circle center
	gtk_snapshot_translate(snapshot, &center);
#if 1
	//Rotate to the scale start degrees
	gtk_snapshot_rotate(snapshot, 360-scale_degrees_start);	//-_- snapshot::rotate uses positive clockwise degrees
	do
	{
		if(scale_degrees_range > ANALOGGAUGE_SCALE_SEGMENT_DEGREE_MAX)
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

		scale_control_degrees = 180*scale_control_radians/M_PI;
#if 0

//		g_print("\ndegrees_start:%f degrees_end:%f degrees_range:%f degrees_rotate:%f\n", scale_degrees_start, scale_degrees_end, scale_degrees_range, scale_control_degrees);
//		g_print("radians_start:%f Ctrl_radians:%f",scale_radians_start, scale_control_radians);
//		g_print("\nRadius:%f: K:%f\nSx :%f Sy :%f\nPx1:%f Py1:%f\nPx2:%f Py2:%f\nEx :%f Ey :%f\n",scale_radius,scale_control_k, scale_start_x,scale_start_y, scale_control_x1, scale_control_y1, scale_control_x2, scale_control_y2, scale_end_x, scale_end_y);

		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	0, 0);
		gsk_path_builder_line_to(builder, 	scale_start_x, scale_start_y);
		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &red);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);

		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	scale_start_x, scale_start_y);
		gsk_path_builder_line_to(builder, 	scale_control_x1, scale_control_y1);
		gsk_path_builder_line_to(builder, 0 ,0);
		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &green);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);

		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	scale_control_x1, scale_control_y1);
		gsk_path_builder_line_to(builder, 	scale_control_x2, scale_control_y2);
		gsk_path_builder_line_to(builder, 0 ,0);
		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &blue);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);

		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	scale_control_x2, scale_control_y2);
		gsk_path_builder_line_to(builder, 	scale_end_x, scale_end_y);
		gsk_path_builder_line_to(builder, 0 ,0);
		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &yellow);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);

#endif
#if 0
		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	0, 0);
		gsk_path_builder_line_to(builder, 	center.x, center.y);
		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &red);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);

		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	center.x, center.y);
		gsk_path_builder_line_to(builder, 	center.x, 0);
		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &red);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);
#endif

		builder = gsk_path_builder_new();
		gsk_path_builder_move_to(builder, 	scale_start_x, scale_start_y);
		gsk_path_builder_cubic_to(builder, 	scale_control_x1, scale_control_y1,
											scale_control_x2, scale_control_y2,
											scale_end_x, scale_end_y);

		path = gsk_path_builder_free_to_path(builder);
		stroke = gsk_stroke_new(5.0f);
		gtk_snapshot_append_stroke(snapshot, path, stroke, &black);
		//gtk_snapshot_append_fill(snapshot, path, GSK_FILL_RULE_WINDING, fill_colors[color_cnt]);
		gsk_stroke_free(stroke);
		gsk_path_unref(path);

		gtk_snapshot_rotate(snapshot, scale_control_degrees);	//rotate past new segment

		scale_radians_start -=scale_control_radians;
		scale_degrees_range -=ANALOGGAUGE_SCALE_SEGMENT_DEGREE_MAX;
		color_cnt++;
		color_cnt%=5;
	}
	while(scale_degrees_range>0);

	scale_degrees_end -=20;
	if(scale_degrees_end < 0)
	{
		scale_degrees_end = 360;
	}
#endif

}

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
  GtkWidget *window;
  GtkWidget *button;
  GtkWidget *vbox;
  GtkWidget *drawingArea;
//  GtkWidget *drawingAreaButton;
  GtkWidgetClass *drawingAreaClass;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Window");
  gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);

  g_signal_connect (window, "destroy", G_CALLBACK (close_window), NULL);
  vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);

  drawingArea = gtk_drawing_area_new();
  gtk_widget_set_size_request (drawingArea, 200, 200);
//  gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(drawing_area), &draw_gauge, NULL, NULL);
  drawingAreaClass = GTK_WIDGET_GET_CLASS(drawingArea);
  drawingAreaClass->snapshot = draw_gauge;
  drawingAreaClass->get_request_mode = drawingArea_get_request_mode;
  drawingAreaClass->measure = drawingArea_measure;

  g_signal_connect (drawingArea, "resize", G_CALLBACK (drawingArea_resize), NULL);

//  drawingAreaButton = gtk_button_new_with_label ("Child Btn!");
//  gtk_widget_set_halign (drawingAreaButton, GTK_ALIGN_CENTER);
//  gtk_widget_set_valign (drawingAreaButton, GTK_ALIGN_CENTER);

  button = gtk_button_new_with_label ("Change Colors!");
  gtk_widget_set_halign (button, GTK_ALIGN_CENTER);
  gtk_widget_set_valign (button, GTK_ALIGN_CENTER);
//
  g_signal_connect (button, "clicked", G_CALLBACK (change_colors_btn), (void *)drawingArea);
//  //g_signal_connect_swapped (button, "clicked", G_CALLBACK (gtk_window_destroy), window);
//
  gtk_box_append(GTK_BOX(vbox), drawingArea);
  gtk_box_append(GTK_BOX(vbox), button);
  gtk_window_set_child (GTK_WINDOW (window), vbox);
// 	  gtk_window_set_child(GTK_WINDOW (window), drawingArea);
  gtk_window_present (GTK_WINDOW (window));
  g_timeout_add(50, (GSourceFunc)change_colors, (gpointer)drawingArea);
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

