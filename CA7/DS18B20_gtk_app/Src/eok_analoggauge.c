/*
 * eok_analoggauge.c
 *
 *  Created on: Apr 1, 2025
 *      Author: seano
 */
#include <gtk/gtk.h>
//#include <gtk/gtkdrawingarea.h>
#include "eok_analoggauge.h"

typedef struct _EokAnalogGaugePrivate EokAnalogGaugePrivate;

struct _EokAnalogGaugePrivate {
	double   scale_radius_offset;
	float scale_value_minimum;
	float scale_value_maximum;
	float scale_degrees_start;
	float scale_degrees_end;
	//TODO: pass pointers?
//	GdkRGBA bezel_color;
//	GdkRGBA scale_color;
};

enum {
	PROP_SCALE_RADIUS_OFFSET = 1,
	PROP_SCALE_VALUE_MINIMUM,
	PROP_SCALE_VALUE_MAXIMUM,
	PROP_SCALE_DEGREES_START,
	PROP_SCALE_DEGREES_END,
//	PROP_BEZEL_COLOR,
//	PROB_SCALE_COLOR,
	LAST_PROP
};

static GParamSpec *props[LAST_PROP] = { NULL, };

enum {
	RESIZE,
	LAST_SIGNAL
};
static guint signals[LAST_SIGNAL] = { 0, };

G_DEFINE_TYPE_WITH_PRIVATE (EokAnalogGauge, eok_analog_gauge, GTK_TYPE_DRAWING_AREA);

static void
eok_analog_gauge_set_property(	GObject      *gobject,
        						guint         prop_id,
								const GValue *value,
								GParamSpec   *pspec)
{
	EokAnalogGauge *self = EOK_ANALOG_GAUGE (gobject);

	switch (prop_id)
	{
		case PROP_SCALE_RADIUS_OFFSET:
			eok_analog_gauge_set_scale_radius_offset(self, g_value_get_double(value));
			break;

		case PROP_SCALE_VALUE_MINIMUM:
			eok_analog_gauge_set_scale_value_minimum(self, g_value_get_float(value));
			break;

		case PROP_SCALE_VALUE_MAXIMUM:
			eok_analog_gauge_set_scale_value_maximum(self, g_value_get_float(value));
			break;

		case PROP_SCALE_DEGREES_START:
			eok_analog_gauge_set_scale_degrees_start(self, g_value_get_float(value));
			break;

		case PROP_SCALE_DEGREES_END:
			eok_analog_gauge_set_scale_degrees_end(self, g_value_get_float(value));
			break;

//		case PROP_BEZEL_COLOR:
//			eok_analog_gauge_set_bezel_color(self, g_value_get_pointer(value));
//			break;
//
//		case PROB_SCALE_COLOR:
//			eok_analog_gauge_set_scale_color(self, g_value_get_pointer(value));
//			break;

		default:
			G_OBJECT_WARN_INVALID_PROPERTY_ID(gobject, prop_id, pspec);
	}
}

static void
eok_analog_gauge_get_property(	GObject    *gobject,
        						guint       prop_id,
								GValue     *value,
								GParamSpec *pspec)
{
	EokAnalogGauge *self = EOK_ANALOG_GAUGE (gobject);
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	switch (prop_id)
	{
	case PROP_SCALE_RADIUS_OFFSET:
	  g_value_set_double (value, priv->scale_radius_offset);
	  break;

	case PROP_SCALE_VALUE_MINIMUM:
	  g_value_set_float (value, priv->scale_value_minimum);
	  break;

	case PROP_SCALE_VALUE_MAXIMUM:
	  g_value_set_float (value, priv->scale_value_maximum);
	  break;

	case PROP_SCALE_DEGREES_START:
	  g_value_set_float (value, priv->scale_degrees_start);
	  break;

	case PROP_SCALE_DEGREES_END:
	  g_value_set_float (value, priv->scale_degrees_end);
	  break;

	default:
	  G_OBJECT_WARN_INVALID_PROPERTY_ID (gobject, prop_id, pspec);
	}
}

static void
eok_analog_gauge_dispose (GObject *object)
{
//  EokAnalogGauge *self = EOK_ANALOG_GAUGE (object);
//  EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);


  G_OBJECT_CLASS (eok_analog_gauge_parent_class)->dispose (object);
}

static GtkSizeRequestMode
eok_analog_gauge_get_request_mode (GtkWidget* widget)
{
	return GTK_SIZE_REQUEST_HEIGHT_FOR_WIDTH;
}

static void
eok_analog_gauge_measure (GtkWidget      *widget,
                          GtkOrientation  orientation,
                          int             for_size,
                          int            *minimum,
                          int            *natural,
                          int            *minimum_baseline,
                          int            *natural_baseline)
{
//  EokAnalogGauge *self = EOK_ANALOG_GAUGE (widget);
//  EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

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

//static void
//eok_analog_gauge_snapshot(	GtkWidget *widget,
//							GtkSnapshot *snapshot)
//{
//	GTK_DRAWING_AREA_CLASS(eok_analog_gauge_partent_class)->snapshot(widget, snapshot);
//}

static void
eok_analog_gauge_draw_func(	EokAnalogGauge *area,
        					cairo_t        *cr,
							int             width,
							int             height,
							gpointer        data)
{
	#define EOK_ANALOG_GAUGE_SCALE_SEGMENT_RADIANS_MAX (M_PI/2)

	float center_x, center_y;
	float bezel_radius;
	float scale_radius, scale_start_x, scale_start_y, scale_end_x, scale_end_y;
	float scale_radians_start, scale_radians_end, scale_radians_range;
	float scale_control_x1, scale_control_y1;
	float scale_control_x2, scale_control_y2;
	float scale_control_radians, scale_control_k;

	float scale_value_range;
	float scale_tick_degrees;

	float scale_radius_offset_from_bezel;
	float scale_value_minimum;
	float scale_value_maximum;
	float scale_degrees_start;
	float scale_degrees_end;

	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (area);
		//Values passed from AnalogGauge Widget
	g_return_if_fail (EOK_IS_ANALOG_GAUGE (area));

	scale_radius_offset_from_bezel 	= priv->scale_radius_offset;
	scale_value_minimum 			= priv->scale_value_minimum;
	scale_value_maximum 			= priv->scale_value_maximum;
	scale_degrees_start 			= priv->scale_degrees_start;	//Always start at a larger degree 2PI = 360
	scale_degrees_end   			= priv->scale_degrees_end;		//Always end at a smaller degree 2PI = 0

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
		if(scale_radians_range > EOK_ANALOG_GAUGE_SCALE_SEGMENT_RADIANS_MAX)
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
		scale_radians_range -=EOK_ANALOG_GAUGE_SCALE_SEGMENT_RADIANS_MAX;
		color_cnt++;
		color_cnt%=5;
	} while(scale_radians_range>0);

	scale_degrees_end -=20;
	if(scale_degrees_end < 0)
	{
		scale_degrees_end = 360;
	}
	eok_analog_gauge_set_scale_degrees_end(area, scale_degrees_end);

	#undef EOK_ANALOG_GAUGE_SCALE_SEGMENT_RADIANS_MAX
}

static void
eok_analog_gauge_class_init (EokAnalogGaugeClass *class)
{
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (class);
  GObjectClass *gobject_class = G_OBJECT_CLASS (class);

  gobject_class->set_property = eok_analog_gauge_set_property;
  gobject_class->get_property = eok_analog_gauge_get_property;
  gobject_class->dispose = eok_analog_gauge_dispose;

  widget_class->measure = eok_analog_gauge_measure;
  widget_class->get_request_mode = eok_analog_gauge_get_request_mode;

  //  GTK_DRAWING_AREA_CLASS (eok_analog_gauge_parent_class)->draw_func = eok_analog_gauge_draw_func;
//  widget_class->size_allocate = gtk_drawing_area_size_allocate;
//  widget_class->snapshot = gtk_drawing_area_snapshot;

  /**
   * EOKAnalogGauge:scale-radius_offset:
   *
   * The scale radius offset from the analog gauge bezel
   */
  props[PROP_SCALE_RADIUS_OFFSET] =
    g_param_spec_double ("scale-radius-offset", NULL, NULL,
                      0, G_MAXDOUBLE, 15,
                      G_PARAM_READWRITE|G_PARAM_EXPLICIT_NOTIFY);

  /**
   * EOKAnalogGauge:scale-value-minimum
   *
   * The minimum value displayed on the scale.
   */
  props[PROP_SCALE_VALUE_MINIMUM] =
    g_param_spec_float ("scale-value-minimum", NULL, NULL,
    				  	G_MINFLOAT, G_MAXFLOAT, 1,
						G_PARAM_READWRITE|G_PARAM_EXPLICIT_NOTIFY);

  /**
   * EOKAnalogGauge:scale-value-maximum
   *
   * The maximum value displayed on the scale.
   */
  props[PROP_SCALE_VALUE_MAXIMUM] =
    g_param_spec_float ("scale-value-maximum", NULL, NULL,
    					G_MINFLOAT, G_MAXFLOAT, 125.0f,
						G_PARAM_READWRITE|G_PARAM_EXPLICIT_NOTIFY);

  /**
   * EOKAnalogGauge:scale-degrees-start
   *
   * The angle in degrees where the scale starts.
   */
  props[PROP_SCALE_DEGREES_START] =
    g_param_spec_float ("scale-degrees-start", NULL, NULL,
    					G_MINFLOAT, G_MAXFLOAT, 240,
						G_PARAM_READWRITE|G_PARAM_EXPLICIT_NOTIFY);

  /**
   * EOKAnalogGauge:scale-degrees-end
   *
   * The angle in degrees where the scale ends.
   */
  props[PROP_SCALE_DEGREES_END] =
    g_param_spec_float ("scale-degrees-end", NULL, NULL,
    					G_MINFLOAT, G_MAXFLOAT, 80,
						G_PARAM_READWRITE|G_PARAM_EXPLICIT_NOTIFY);

  g_object_class_install_properties (gobject_class, LAST_PROP, props);

  /**
   * GtkDrawingArea::resize:
   * @area: the `GtkDrawingArea` that emitted the signal
   * @width: the width of the viewport
   * @height: the height of the viewport
   *
   * Emitted once when the widget is realized, and then each time the widget
   * is changed while realized.
   *
   * This is useful in order to keep state up to date with the widget size,
   * like for instance a backing surface.
   */
//  signals[RESIZE] =
//    g_signal_new (I_("resize"),
//                  G_TYPE_FROM_CLASS (class),
//                  G_SIGNAL_RUN_LAST,
//                  G_STRUCT_OFFSET (GtkDrawingAreaClass, resize),
//                  NULL, NULL,
//                  _gdk_marshal_VOID__INT_INT,
//                  G_TYPE_NONE, 2, G_TYPE_INT, G_TYPE_INT);
//  g_signal_set_va_marshaller (signals[RESIZE],
//                              G_TYPE_FROM_CLASS (class),
//                              _gdk_marshal_VOID__INT_INTv);
}

static void
eok_analog_gauge_init (EokAnalogGauge *agauge)
{
//  gtk_widget_set_focusable (GTK_WIDGET (darea), FALSE);
	gtk_drawing_area_set_draw_func( GTK_DRAWING_AREA(agauge), (GtkDrawingAreaDrawFunc)eok_analog_gauge_draw_func, NULL, NULL);
}

GtkWidget*
eok_analog_gauge_new (void)
{
  return g_object_new (EOK_TYPE_ANALOG_GAUGE, NULL);
}

void eok_analog_gauge_set_scale_radius_offset(EokAnalogGauge	*self,
                                                 double         offset)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_if_fail (EOK_IS_ANALOG_GAUGE (self));
	g_return_if_fail (offset >= 0);

	if (priv->scale_radius_offset == offset)
	return;

	priv->scale_radius_offset = offset;

	gtk_widget_queue_resize (GTK_WIDGET (self));
	g_object_notify_by_pspec (G_OBJECT (self), props[PROP_SCALE_RADIUS_OFFSET]);
}

double eok_analog_gauge_get_scale_radius_offset(EokAnalogGauge *self)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_val_if_fail (EOK_IS_ANALOG_GAUGE (self), 0);

	return priv->scale_radius_offset;
}

void eok_analog_gauge_set_scale_value_minimum(EokAnalogGauge	*self,
                                                 float         	value)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_if_fail (EOK_IS_ANALOG_GAUGE (self));

	if (priv->scale_value_minimum == value)
	return;

	priv->scale_value_minimum = value;

	gtk_widget_queue_resize (GTK_WIDGET (self));
	g_object_notify_by_pspec (G_OBJECT (self), props[PROP_SCALE_VALUE_MINIMUM]);
}

float eok_analog_gauge_get_scale_value_minimum(EokAnalogGauge *self)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_val_if_fail (EOK_IS_ANALOG_GAUGE (self), 0);

	return priv->scale_value_minimum;
}

void eok_analog_gauge_set_scale_value_maximum(EokAnalogGauge	*self,
                                                 float         	value)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_if_fail (EOK_IS_ANALOG_GAUGE (self));

	if (priv->scale_value_maximum == value)
	return;

	priv->scale_value_maximum= value;

	gtk_widget_queue_resize (GTK_WIDGET (self));
	g_object_notify_by_pspec (G_OBJECT (self), props[PROP_SCALE_VALUE_MAXIMUM]);
}

float eok_analog_gauge_get_scale_value_maximum(EokAnalogGauge *self)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_val_if_fail (EOK_IS_ANALOG_GAUGE (self), 0);

	return priv->scale_value_maximum;
}


void eok_analog_gauge_set_scale_degrees_start(EokAnalogGauge	*self,
                                                 float         	value)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_if_fail (EOK_IS_ANALOG_GAUGE (self));

	if (priv->scale_degrees_start == value)
	return;

	priv->scale_degrees_start= value;

	gtk_widget_queue_resize (GTK_WIDGET (self));
	g_object_notify_by_pspec (G_OBJECT (self), props[PROP_SCALE_DEGREES_START]);
}

float eok_analog_gauge_get_scale_degrees_start(EokAnalogGauge *self)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_val_if_fail (EOK_IS_ANALOG_GAUGE (self), 0);

	return priv->scale_value_maximum;
}


void eok_analog_gauge_set_scale_degrees_end(EokAnalogGauge	*self,
                                                 float         	value)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_if_fail (EOK_IS_ANALOG_GAUGE (self));

	if (priv->scale_degrees_end == value)
	return;

	priv->scale_degrees_end= value;

	gtk_widget_queue_resize (GTK_WIDGET (self));
	g_object_notify_by_pspec (G_OBJECT (self), props[PROP_SCALE_DEGREES_END]);
}

float eok_analog_gauge_get_scale_degrees_end(EokAnalogGauge *self)
{
	EokAnalogGaugePrivate *priv = eok_analog_gauge_get_instance_private (self);

	g_return_val_if_fail (EOK_IS_ANALOG_GAUGE (self), 0);

	return priv->scale_degrees_end;
}
