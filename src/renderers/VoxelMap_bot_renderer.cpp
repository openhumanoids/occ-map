/*
 * renders a VoxelMap
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <gtk/gtk.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <bot_core/bot_core.h>
#include <bot_vis/bot_vis.h>

#include <occ_map/VoxelMap.hpp>

#include <lcmtypes/occ_map_voxel_map_t.h>

#include "occ_map_renderers.h"

#define RENDERER_NAME "VoxelMap"
#define PARAM_COLOR_MODE "Color Mode"
#define PARAM_COLOR_MODE_Z_MAX_Z "Red Height"
#define PARAM_COLOR_MODE_Z_MIN_Z "Blue Height"
#define PARAM_SHOW_FREE "Show Free"
#define PARAM_SHOW_OCC  "Show Occ"
#define PARAM_SHOW_ALPHA "Show Alpha"
#define PARAM_CUTOFF "Occ Cutoff"
#define PARAM_POINT_SIZE "Point Size"

using namespace occ_map;

typedef enum _color_mode_t {
  COLOR_MODE_DRAB, COLOR_MODE_Z,
} color_mode_t;

typedef struct _OccMapRendererVoxelMap OccMapRendererVoxelMap;

struct _OccMapRendererVoxelMap {
  BotRenderer renderer;
  BotEventHandler ehandler;
  lcm_t *lc;
  BotGtkParamWidget *pw;
  GtkWidget *label;
  BotViewer *viewer;
  FloatVoxelMap* voxmap;

  //vertex buffer to draw all points at once
  int pointBuffSize;
  double * pointBuffer;
  float * colorBuffer;

  int numPointsToDraw;

};

static void update_vertex_buffers(OccMapRendererVoxelMap *self)
{
  bool show_free = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_FREE);
  bool show_occ = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_OCC);
  bool show_alpha = bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_ALPHA);

  double cutoff = bot_gtk_param_widget_get_double(self->pw, PARAM_CUTOFF);

  int color_mod = bot_gtk_param_widget_get_enum(self->pw, PARAM_COLOR_MODE);

  self->numPointsToDraw = 0;
  int ixyz[3];
  for (ixyz[2] = 0; ixyz[2] < self->voxmap->dimensions[2]; ixyz[2]++) {
    for (ixyz[1] = 0; ixyz[1] < self->voxmap->dimensions[1]; ixyz[1]++) {
      for (ixyz[0] = 0; ixyz[0] < self->voxmap->dimensions[0]; ixyz[0]++) {
        float likelihood = self->voxmap->readValue(ixyz);
//        if (likelihood>.01 && likelihood<.99)
//          fprintf(stderr,"%f ",likelihood);
        if ((show_occ && likelihood >= 0 && likelihood > cutoff) || (show_free && likelihood >= 0 && likelihood
            < cutoff)) {
          self->numPointsToDraw++;
        }

      }
    }
  }
  fprintf(stderr, "there are %d points to draw\n", self->numPointsToDraw);
  self->pointBuffSize = self->numPointsToDraw;

  int color_size = 3;
  if (show_alpha)
    color_size = 4;
  self->pointBuffer = (double *) realloc(self->pointBuffer, self->numPointsToDraw * 3 * sizeof(double));
  self->colorBuffer = (float *) realloc(self->colorBuffer, self->numPointsToDraw * color_size * sizeof(float));

  double min_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z);
  double max_z = bot_gtk_param_widget_get_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z);

  int numLoadedPoints = 0;
  for (ixyz[2] = 0; ixyz[2] < self->voxmap->dimensions[2]; ixyz[2]++) {
    for (ixyz[1] = 0; ixyz[1] < self->voxmap->dimensions[1]; ixyz[1]++) {
      for (ixyz[0] = 0; ixyz[0] < self->voxmap->dimensions[0]; ixyz[0]++) {
        float likelihood = self->voxmap->readValue(ixyz);
        if ((show_occ && likelihood >= 0 && likelihood > cutoff) || (show_free && likelihood >= 0 && likelihood
            < cutoff)) {
          double * pointP = self->pointBuffer + (3 * numLoadedPoints); //pointer into the vertex buffer
          self->voxmap->tableToWorld(ixyz, pointP);
          float * colorP = self->colorBuffer + (color_size * numLoadedPoints); //pointer into the color buffer
          switch (color_mod) {
          case COLOR_MODE_Z:
            {
              double z_norm = (pointP[2] - min_z) / (max_z - min_z);
              float * jetC = bot_color_util_jet(z_norm);
              memcpy(colorP, jetC, 3 * sizeof(float));
            }
            break;
          case COLOR_MODE_DRAB:
            {
              colorP[0] = 0.3;
              colorP[1] = 0.3;
              colorP[2] = 0.3;
            }
            break;
          }
          if (show_alpha)
            colorP[3] = (likelihood-cutoff)/(1-cutoff);

          numLoadedPoints++;
        }
      }
    }
  }
}

static void voxmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, 
        const occ_map_voxel_map_t *msg, void *user)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) user;

  if (self->voxmap != NULL)
    delete self->voxmap;
  self->voxmap = new FloatVoxelMap(msg);
  update_vertex_buffers(self);
  fprintf(stderr, "M");
}

static void VoxelMap_draw(BotViewer *viewer, BotRenderer *renderer)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) renderer;
  if (self->voxmap == NULL)
    return;

  glPushAttrib(GL_DEPTH_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
  //z_buffering
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glPointSize(bot_gtk_param_widget_get_int(self->pw, PARAM_POINT_SIZE));

  //render using vertex buffers
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_COLOR_ARRAY);

  int color_size = 3;
  if (bot_gtk_param_widget_get_bool(self->pw, PARAM_SHOW_ALPHA))
    color_size = 4;

  glColorPointer(color_size, GL_FLOAT, 0, self->colorBuffer);
  glVertexPointer(3, GL_DOUBLE, 0, self->pointBuffer);
  //draw
  glDrawArrays(GL_POINTS, 0, self->numPointsToDraw);

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_COLOR_ARRAY);

  glPopAttrib();

}

static void VoxelMap_free(BotRenderer *renderer)
{
  //TODO: don't LEAK!
  free(renderer);
}

static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) user;
  if (self->voxmap != NULL)
    update_vertex_buffers(self);
  bot_viewer_request_redraw(self->viewer);
}

static BotRenderer* 
renderer_voxel_map_new(BotViewer *viewer, int render_priority, const char* lcm_channel)
{
  OccMapRendererVoxelMap *self = (OccMapRendererVoxelMap*) calloc(1, sizeof(OccMapRendererVoxelMap));
  BotRenderer *renderer = &self->renderer;
  self->viewer = viewer;
  self->lc = bot_lcm_get_global(NULL);

  renderer->draw = VoxelMap_draw;
  renderer->destroy = VoxelMap_free;
  renderer->widget = gtk_vbox_new(FALSE, 0);
  renderer->name = (char *) RENDERER_NAME;
  renderer->user = self;
  renderer->enabled = 1;

  self->pw = BOT_GTK_PARAM_WIDGET(bot_gtk_param_widget_new());
  renderer->widget = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start(GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
  bot_gtk_param_widget_add_enum(self->pw, PARAM_COLOR_MODE, BOT_GTK_PARAM_WIDGET_MENU, 
          0, "Height", COLOR_MODE_Z,
          "Drab", COLOR_MODE_DRAB, NULL);

  bot_gtk_param_widget_add_int(self->pw, PARAM_POINT_SIZE, BOT_GTK_PARAM_WIDGET_SLIDER, 1, 20, 1, 4);

  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_ALPHA, 0, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_OCC, 1, NULL);
  bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_FREE, 0, NULL);

  bot_gtk_param_widget_add_double(self->pw, PARAM_CUTOFF, BOT_GTK_PARAM_WIDGET_SLIDER, 0, 1, .01, .1);

  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MAX_Z, BOT_GTK_PARAM_WIDGET_SLIDER, -2, 40, .5, 15);
  bot_gtk_param_widget_add_double(self->pw, PARAM_COLOR_MODE_Z_MIN_Z, BOT_GTK_PARAM_WIDGET_SLIDER, -2, 40, .5, 0);

  GtkWidget *clear_button = gtk_button_new_with_label("Clear memory");
  gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button, FALSE, FALSE, 0);

  gtk_widget_show_all(renderer->widget);
  g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
  on_param_widget_changed(self->pw, "", self);

  // pick a default channel if none specified
  if(!lcm_channel || !strlen(lcm_channel))
    lcm_channel = "VOXEL_MAP";
  occ_map_voxel_map_t_subscribe(self->lc, lcm_channel, voxmap_handler, self);
  return &self->renderer;
}

extern "C" 
void 
occ_map_voxel_map_add_renderer_to_viewer(BotViewer *viewer, int render_priority, const char* lcm_channel)
{
  BotRenderer* renderer = renderer_voxel_map_new(viewer, render_priority, lcm_channel);
  bot_viewer_add_renderer(viewer, renderer, render_priority);
}
