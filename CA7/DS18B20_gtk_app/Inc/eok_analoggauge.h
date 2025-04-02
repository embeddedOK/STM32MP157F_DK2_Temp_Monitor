/*
 * eokanaloggauge.h
 *
 *  Created on: Mar 27, 2025
 *      Author: seanok
 *
 *      Widget inheriting GtkDrawingArea used to display an Analog Gauge on the screen
 */
#ifndef INC_EOKANALOGGAUGE_H_
#define INC_EOKANALOGGAUGE_H_

//#if !defined (__GTK_H_INSIDE__) && !defined (GTK_COMPILATION)
//#error "Only <gtk/gtk.h> can be included directly."
//#endif

#include <gtk/gtk.h>
//#include <gtk/gtkdrawingarea.h>

G_BEGIN_DECLS
#define EOK_TYPE_ANALOG_GAUGE            (eok_analog_gauge_get_type ())
#define EOK_ANALOG_GAUGE(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), EOK_TYPE_ANALOG_GAUGE, EokAnalogGauge))
#define EOK_ANALOG_GAUGE_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass),  EOK_TYPE_ANALOG_GAUGE, EokAnalogGaugeClass))
#define EOK_IS_ANALOG_GAUGE(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), EOK_TYPE_ANALOG_GAUGE))
#define EOK_IS_ANALOG_GAUGE_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass),  EOK_TYPE_ANALOG_GAUGE))
#define EOK_ANALOG_GAUGE_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj),  EOK_TYPE_ANALOG_GAUGE, EokAnalogGaugeClass))

typedef struct _EokAnalogGauge       EokAnalogGauge;
typedef struct _EokAnalogGaugeClass  EokAnalogGaugeClass;


struct _EokAnalogGauge
{
  GtkDrawingArea widget;
};

struct _EokAnalogGaugeClass
{
  GtkDrawingAreaClass parent_class;


  /*< private >*/

  gpointer padding[8];
};

GDK_AVAILABLE_IN_ALL
GType      eok_analog_gauge_get_type (void) G_GNUC_CONST;
GDK_AVAILABLE_IN_ALL
GtkWidget* eok_analog_gauge_new      (void);

GDK_AVAILABLE_IN_ALL
void 	eok_analog_gauge_set_scale_radius_offset(EokAnalogGauge	*self,
                                                 double         offset);
GDK_AVAILABLE_IN_ALL
double	eok_analog_gauge_get_scale_radius_offset(EokAnalogGauge *self);

GDK_AVAILABLE_IN_ALL
void 	eok_analog_gauge_set_scale_value_minimum(EokAnalogGauge	*self,
                                                 float         	value);
GDK_AVAILABLE_IN_ALL
float	eok_analog_gauge_get_scale_value_minimum(EokAnalogGauge *self);

GDK_AVAILABLE_IN_ALL
void 	eok_analog_gauge_set_scale_value_maximum(EokAnalogGauge	*self,
                                                 float         	value);
GDK_AVAILABLE_IN_ALL
float	eok_analog_gauge_get_scale_value_maximum(EokAnalogGauge *self);

GDK_AVAILABLE_IN_ALL
void 	eok_analog_gauge_set_scale_degrees_start(EokAnalogGauge	*self,
                                                 float         	value);
GDK_AVAILABLE_IN_ALL
float	eok_analog_gauge_get_scale_degrees_start(EokAnalogGauge *self);

GDK_AVAILABLE_IN_ALL
void 	eok_analog_gauge_set_scale_degrees_end(EokAnalogGauge	*self,
                                                 float         	value);
GDK_AVAILABLE_IN_ALL
float	eok_analog_gauge_get_scale_degrees_end(EokAnalogGauge *self);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(EokAnalogGauge, g_object_unref)

G_END_DECLS
#endif /* INC_GTKANALOGGAUGE_H_ */
