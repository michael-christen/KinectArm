#include "eecs467_util.h"

struct eecs467_default_implementation
{
    vx_world_t *world;
    pthread_mutex_t mutex;

    zhash_t *layers;
};

eecs467_default_implementation_t *eecs467_default_implementation_create(vx_world_t *world, zhash_t *layers)
{
    eecs467_default_implementation_t *impl = calloc(1, sizeof(eecs467_default_implementation_t));
    impl->world = world;
	impl->layers = layers;

    pthread_mutex_init(&impl->mutex, NULL);

    return impl;
}

void eecs467_default_display_started(vx_application_t *app, vx_display_t *disp)
{
    eecs467_default_implementation_t *impl = app->impl;

    vx_layer_t *layer = vx_layer_create(impl->world);
    vx_layer_set_display(layer, disp);

    pthread_mutex_lock(&impl->mutex);
    // store a reference to the world and layer that we associate with each
    // vx_display_t
    zhash_put(impl->layers, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&impl->mutex);
}

void eecs467_default_display_finished(vx_application_t *app, vx_display_t *disp)
{
    eecs467_default_implementation_t *impl = app->impl;
    pthread_mutex_lock(&impl->mutex);

    vx_layer_t *layer = NULL;
    zhash_remove(impl->layers, &disp, NULL, &layer);
    vx_layer_destroy(layer);

    pthread_mutex_unlock(&impl->mutex);
}

void eecs467_init(int argc, char **argv)
{
    // on newer GTK systems, this might generate an error/warning
    g_type_init();

    // Initialize GTK
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    vx_global_init();
}

void eecs467_gui_run(vx_application_t *app, parameter_gui_t *pg, int w, int h)
{
    // Creates a GTK window to wrap around our vx display canvas. The vx world
    // is rendered to the canvas widget, which acts as a viewport into your
    // virtual world.
    vx_gtk_display_source_t *appwrap = vx_gtk_display_source_create(app);
    GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    GtkWidget *canvas = vx_gtk_display_source_get_widget(appwrap);
    GtkWidget *pgui = pg_get_widget(pg);
    gtk_window_set_default_size(GTK_WINDOW(window), w, h);

    // Pack a parameter gui and canvas into a vertical box
    GtkWidget *vbox = (GtkWidget*)gtk_vbox_new(0, 0);
    gtk_box_pack_start(GTK_BOX(vbox), canvas, 1, 1, 0);
    gtk_widget_show(canvas);    // XXX Show all causes errors!
    gtk_box_pack_start(GTK_BOX(vbox), pgui, 0, 0, 0);
    gtk_widget_show(pgui);

    gtk_container_add(GTK_CONTAINER(window), vbox);
    gtk_widget_show(window);
    gtk_widget_show(vbox);

    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main(); // Blocks as long as GTK window is open
    gdk_threads_leave();

    vx_gtk_display_source_destroy(appwrap);
    printf("after disp destroy leave\n");
}
