/*
 * gtkanaloggauge.h
 *
 *  Created on: Mar 27, 2025
 *      Author: seano
 */

#ifndef INC_GTKANALOGGAUGE_H_
#define INC_GTKANALOGGAUGE_H_

#include <gtk/gtkwidget.h>

G_BEGIN_DECLS
#define GTK_TYPE_ANALOG_GAUGE            (gtk_analog_gauge_get_type ())
#define GTK_ANALOG_GAUGE(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_TYPE_ANALOG_GAUGE, GtkAnalogGauge))
#define GTK_ANALOG_GAUGE_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), GTK_TYPE_ANALOG_GAUGE, GtkAnalogGaugeClass))
#define GTK_IS_ANALOG_GAUGE(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_TYPE_ANALOG_GAUGE))
#define GTK_IS_ANALOG_GAUGE_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), GTK_TYPE_ANALOG_GAUGE))
#define GTK_ANALOG_GAUGE_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_TYPE_ANALOG_GAUGE, GtkAnalogGaugeClass))

typedef struct _GtkAnalogGauge       GtkAnalogGauge;
typedef struct _GtkAnalogGaugeClass  GtkAnalogGaugeClass;


struct _GtkAnalogGauge
{
  GtkWidget widget;
};

struct _GtkAnalogGaugeClass
{
  GtkWidgetClass parent_class;

  void           (* resize)         (GtkDrawingArea *area,
                                     int             width,
                                     int             height);

  /*< private >*/

  gpointer padding[8];
};

GDK_AVAILABLE_IN_ALL
GType      gtk_analog_gauge_get_type (void) G_GNUC_CONST;
GDK_AVAILABLE_IN_ALL
GtkWidget* gtk_analog_gauge_new      (void);

GDK_AVAILABLE_IN_ALL
void            gtk_analog_gauge_set_content_radius      (GtkDrawingArea         *self,
                                                         int                     radius);
GDK_AVAILABLE_IN_ALL
int             gtk_analog_gauge_get_content_radius      (GtkDrawingArea         *self);
GDK_AVAILABLE_IN_ALL
void            gtk_analog_gauge_set_content_height     (GtkDrawingArea         *self,
                                                         int                     height);
GDK_AVAILABLE_IN_ALL
int             gtk_analog_gauge_get_content_height     (GtkDrawingArea         *self);
GDK_AVAILABLE_IN_ALL
void            gtk_analog_gauge_set_draw_func          (GtkDrawingArea         *self,
                                                         GtkDrawingAreaDrawFunc  draw_func,
                                                         gpointer                user_data,
                                                         GDestroyNotify          destroy);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(GtkAnalogGague, g_object_unref)

G_END_DECLS
#endif /* INC_GTKANALOGGAUGE_H_ */
