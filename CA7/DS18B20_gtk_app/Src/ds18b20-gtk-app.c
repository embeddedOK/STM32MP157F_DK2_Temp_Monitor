#include <gtk/gtk.h>
#include "eok_analoggauge.h"
//#include <math.h>

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

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
  GtkWidget *window;
  GtkWidget *button;
  GtkWidget *vbox;
  GtkWidget *analogGauge;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Window");
  gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);

  g_signal_connect (window, "destroy", G_CALLBACK (close_window), NULL);
  vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);

  analogGauge = eok_analog_gauge_new();

  eok_analog_gauge_set_scale_radius_offset(EOK_ANALOG_GAUGE(analogGauge), 15);
  eok_analog_gauge_set_scale_value_minimum(EOK_ANALOG_GAUGE(analogGauge), -40.0f);
  eok_analog_gauge_set_scale_value_maximum(EOK_ANALOG_GAUGE(analogGauge), 125.0f);
  eok_analog_gauge_set_scale_degrees_start(EOK_ANALOG_GAUGE(analogGauge), 270.0f);
  eok_analog_gauge_set_scale_degrees_end(EOK_ANALOG_GAUGE(analogGauge), 90.0f);

  gtk_widget_set_size_request (analogGauge, 200, 200);

  button = gtk_button_new_with_label ("Change Colors!");
  gtk_widget_set_halign (button, GTK_ALIGN_CENTER);
  gtk_widget_set_valign (button, GTK_ALIGN_CENTER);

  g_signal_connect (button, "clicked", G_CALLBACK (change_colors_btn), (void *)analogGauge);

  gtk_box_append(GTK_BOX(vbox), analogGauge);
  gtk_box_append(GTK_BOX(vbox), button);
  gtk_window_set_child (GTK_WINDOW (window), vbox);

  gtk_window_present (GTK_WINDOW (window));
//  g_timeout_add(100, (GSourceFunc)change_colors, (gpointer)analogGauge);
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

