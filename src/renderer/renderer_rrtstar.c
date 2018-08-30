#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef __APPLE__
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <gdk/gdkkeysyms.h>

#include <lcm/lcm.h>

#include <bot_core/bot_core.h>
#include <bot_vis/gtk_util.h>
#include <bot_vis/viewer.h>
#include <bot_vis/gl_util.h>

#include <geom_utils/geometry.h>

#include <lcmtypes/hr_lcmtypes.h>
#include <lcmtypes/ripl_rrt_command_t.h>
#include <lcmtypes/ripl_rrt_traj_t.h>
#include <lcmtypes/ripl_rrt_tree_t.h>
#include <lcmtypes/bot_core_pose_t.h>

#define RENDERER_NAME "RRT_STAR"
#define MAX_HISTORY 1000

#define SLIDER_HISTORY "History"
#define CHECKBOX_RENDER_TRAJ "Render Best Trajectory"
#define CHECKBOX_RENDER_FOOTPRINT "Render Footprint"
#define CHECKBOX_RENDER_TREE "Render the Tree"
#define CHECKBOX_DO_HEADING "Consider goal heading"
#define CHECKBOX_RENDER_COSTS "Render path costs"

#define DRAW_PERSIST_SEC 4


typedef struct _region_2d_t {
    double center[2];
    double size[2];
} region_2d_t;


typedef struct _RendererRRTStar {
    BotRenderer renderer;    
    BotEventHandler ehandler;
    lcm_t *lcm; 
    BotViewer *viewer;    
    BotGtkParamWidget *pw;

  
    int dragging;
    point2d_t drag_start_local;
    point2d_t drag_finish_local;
    
    point2d_t goal_mean;
    double goal_theta;
    double goal_std;
    
    int64_t max_draw_utime;
    gboolean do_heading;
    
    int index_trees;
    int num_trees;
    ripl_rrt_tree_t *trees[MAX_HISTORY];
    ripl_rrt_traj_t *traj;
    int slider_history_no;
    gboolean use_const_nodes;
    int num_const_nodes;
    gboolean run_mp_rrt_rrt;

    int tree_pub_final;
    int tree_pub_interval;
    int tree_pub_interval_iteration;

    int traj_pub_final;
    int traj_pub_interval;
    int traj_pub_interval_iteration;

    gboolean render_traj;
    gboolean render_tree;
    gboolean render_footprint;
    gboolean render_costs;

    GMutex *mutex;

    double mouseNodeXY[2];

    // Goal region
    int key_goal;
    region_2d_t region_goal;
    region_2d_t last_region_goal;

    bot_core_pose_t *bot_pose_last;

    double last_xy[2];   // Used moving and resizing operating, goal, and obstacle regions

    double footprint[8]; // Vehicle footprint

    // Pointers to the modify buttons
    GtkWidget *modify_goal_button;

    GtkWidget *display_last_env_button;

} RendererRRTStar;


void 
rrtstar_renderer_draw (BotViewer *viewer, BotRenderer *renderer);



static void
recompute_particle_distribution(RendererRRTStar *self)
{
  self->goal_mean = self->drag_start_local;
  double dx = self->drag_finish_local.x - self->drag_start_local.x;
  double dy = self->drag_finish_local.y - self->drag_start_local.y;

  double theta = atan2(dy,dx);
  self->goal_theta = theta;

  self->goal_std = sqrt(dx*dx + dy*dy);

  self->max_draw_utime = bot_timestamp_now() + DRAW_PERSIST_SEC * 1000000;
}

void 
on_param_widget_changed (BotGtkParamWidget *pw, const char *name,
                         RendererRRTStar *self) {

    self->slider_history_no = bot_gtk_param_widget_get_int (self->pw, SLIDER_HISTORY);
    
    self->render_tree = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_RENDER_TREE);
    self->render_traj = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_RENDER_TRAJ);
    self->render_footprint = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_RENDER_FOOTPRINT);
    self->do_heading = bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_DO_HEADING);

    if (!self->render_tree) {
        bot_gtk_param_widget_set_enabled (self->pw, CHECKBOX_RENDER_COSTS, 0);
        bot_gtk_param_widget_set_bool (self->pw, CHECKBOX_RENDER_COSTS, 0);
    }
    else
        bot_gtk_param_widget_set_enabled (self->pw, CHECKBOX_RENDER_COSTS, 1);

    self->render_costs =  bot_gtk_param_widget_get_bool (self->pw, CHECKBOX_RENDER_COSTS);

    bot_viewer_request_redraw(self->viewer);    
    
    return;
}


static void
on_clear_button(GtkWidget *button, RendererRRTStar *self)
{

    g_mutex_lock (self->mutex);


    for (int i = 0; i < MAX_HISTORY; i++) 
        if (self->trees[i]) {
            ripl_rrt_tree_t_destroy (self->trees[i]);
            self->trees[i] = NULL;
        }
    self->num_trees = 0;
    self->index_trees = 0;
    self->slider_history_no = 1;
    
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_HISTORY, 0);
    

    bot_gtk_param_widget_modify_int (self->pw, SLIDER_HISTORY, 
                                     1, 2,
                                     1, self->slider_history_no);

    g_mutex_unlock (self->mutex);

    bot_viewer_request_redraw(self->viewer);
}


static void
on_start_button (GtkWidget *button, RendererRRTStar *self)
{
    ripl_goal_list_t goal_list; 

    goal_list.utime = bot_timestamp_now();
    goal_list.sender_id = 0;
    goal_list.num_goals = 1;
    goal_list.goals = calloc(1,sizeof(ripl_goal_t));
    goal_list.goals[0].pos[0] = self->goal_mean.x;
    goal_list.goals[0].pos[1] = self->goal_mean.y;

    //Sachi - hack for now until we can get the heading 

    goal_list.goals[0].theta = self->goal_theta; //M_PI/2;

    goal_list.goals[0].do_turn_only = 0;
    
    if(self->do_heading){      
      goal_list.goals[0].heading_tol = 0.3; 
      goal_list.goals[0].use_theta = 1;
    }
    else{
      goal_list.goals[0].heading_tol = 2*M_PI; 
      goal_list.goals[0].use_theta = 0;
    }
    goal_list.goals[0].size[0] = self->goal_std*2;//self->region_goal.size[0];
    goal_list.goals[0].size[1] = self->goal_std*2;//self->region_goal.size[0];

    ripl_goal_list_t_publish (self->lcm, "RRTSTAR_GOALS", &goal_list);
    free(goal_list.goals);
    
    bot_viewer_request_redraw(self->viewer);
}



static void
on_modify_goal_button (GtkWidget *button, RendererRRTStar *self)
{
    
    g_mutex_lock (self->mutex);

    // Set the key_* variables
    self->key_goal = 1;

    bot_viewer_request_pick (self->viewer, &(self->ehandler));
    
    // Enable/Disable the buttons
    gtk_widget_set_sensitive (self->modify_goal_button, FALSE);

    g_mutex_unlock (self->mutex);

    bot_viewer_request_redraw(self->viewer);
}

static void
on_mp_rrt_tree (const lcm_recv_buf_t *buf, const char *channel,
                 const ripl_rrt_tree_t *msg, void *user) {
    RendererRRTStar *self = (RendererRRTStar *)user;

//     printf ("Got tree message \n");
//     printf ("Num edges : %d \n", msg->num_edges);

    g_mutex_lock (self->mutex);


    if (self->trees[self->index_trees])
        ripl_rrt_tree_t_destroy (self->trees[self->index_trees]);
    self->trees[self->index_trees] = ripl_rrt_tree_t_copy (msg);
    self->index_trees++;
    if (self->index_trees >= MAX_HISTORY)
        self->index_trees = 0;
    
    self->num_trees++;
    if (self->num_trees >= MAX_HISTORY)
        self->num_trees = MAX_HISTORY;

    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_HISTORY, 1);

    bot_gtk_param_widget_modify_int (self->pw, SLIDER_HISTORY, 
                                     1, self->num_trees,
                                     1, self->slider_history_no);

    g_mutex_unlock (self->mutex);

    bot_viewer_request_redraw(self->viewer);
}


static void
on_mp_rrt_traj (const lcm_recv_buf_t *buf, const char *channel,
                const ripl_rrt_traj_t *msg, void *user) {
    RendererRRTStar *self = (RendererRRTStar *)user;

    g_mutex_lock (self->mutex);
    
    if (self->traj) 
        ripl_rrt_traj_t_destroy (self->traj);
    self->traj = ripl_rrt_traj_t_copy (msg);
    
    g_mutex_unlock (self->mutex);

    bot_viewer_request_redraw(self->viewer);

}


static void
on_mp_rrt_cmd (const lcm_recv_buf_t *buf, const char *channel,
               const ripl_rrt_command_t *msg, void *user) {
    RendererRRTStar *self = (RendererRRTStar *)user;

    bot_viewer_request_redraw(self->viewer);

}



static void
on_bot_pose (const lcm_recv_buf_t *buf, const char *channel,
             const bot_core_pose_t *msg, void *user) {
    
    RendererRRTStar *self = (RendererRRTStar *)user;
    
    g_mutex_lock (self->mutex);

    if (self->bot_pose_last)
        bot_core_pose_t_destroy (self->bot_pose_last);
    self->bot_pose_last = bot_core_pose_t_copy (msg);

    g_mutex_unlock (self->mutex);
}


void
rrtstar_renderer_destroy (BotRenderer *renderer) {

    return;
}



static int 
mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
             const double ray_dir[3], const GdkEventButton *event)
{

  RendererRRTStar *self = (RendererRRTStar *)ehandler->user;

  self->dragging = 0;

  // Is this mouse_press event meant for us?
  if (event->button==1 && self->key_goal==1 && ehandler->picking) {

      point2d_t click_pt_local;
  
      if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
                                             POINT3D(ray_dir), 0, &click_pt_local)) {
          bot_viewer_request_redraw(self->viewer);
          self->key_goal = 0;
          return 0;
      }

      self->dragging = 1;

      self->drag_start_local = click_pt_local;
      self->drag_finish_local = click_pt_local;

      recompute_particle_distribution(self);
      
      bot_viewer_request_redraw(self->viewer);
      return 1;
  } else
      return 0;

}

static int mouse_release(BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventButton *event)
{

  RendererRRTStar *self = (RendererRRTStar *)ehandler->user;

  if (self->dragging) {
      self->dragging = 0;    
  }

  // If the mouse_release event wasn't meant for us
  if (!self->key_goal) {
      ehandler->picking = 0;
      self->dragging = 0;
      return 0;
  }
  else {
      self->key_goal = 0;
      fprintf(stderr,"RRTStar Goal Yaw : %f\n", self->goal_theta);
      //set the button active
      gtk_widget_set_sensitive (self->modify_goal_button, TRUE);
      return 1;
  }
}


static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,
                         const double ray_start[3], const double ray_dir[3], 
                         const GdkEventMotion *event)
{
  RendererRRTStar *self = (RendererRRTStar*) ehandler->user;

  // If we aren't dragging with a click or in goal modification
  // mode, simply return since this isn't meant for us
  if(!self->dragging || self->key_goal==0)
    return 0;

  point2d_t drag_pt_local;
  if (0 != geom_ray_z_plane_intersect_3d(POINT3D(ray_start), 
        POINT3D(ray_dir), 0, &drag_pt_local)) {
    return 0;
  }

  self->drag_finish_local = drag_pt_local;
  recompute_particle_distribution(self);

  bot_viewer_request_redraw(self->viewer);
  return 1;
}



static int key_press (BotViewer *viewer, BotEventHandler *ehandler, 
                      const GdkEventKey *event)
{
    RendererRRTStar *self = (RendererRRTStar*) ehandler->user;

    if (event->keyval == 'd' || event->keyval == 'D') {
        if (!self->key_goal) {            
            bot_viewer_request_pick(viewer, ehandler);
            self->key_goal = 1;

            // Disable the button
            gtk_widget_set_sensitive (self->modify_goal_button, FALSE);
            return 1;
        }
        else {
            ehandler->picking = 0;
            self->key_goal = 0;
            gtk_widget_set_sensitive (self->modify_goal_button, TRUE);
        }
    }

    // Was the ESC meant for this function?
    if (self->key_goal && event->keyval == GDK_Escape) {
        ehandler->picking = 0;
        self->key_goal = 0;

        gtk_widget_set_sensitive (self->modify_goal_button, TRUE);

        return 1;
    }

    return 0;
}

static void
draw_footprint (RendererRRTStar * self)
{ 
    float color_curr[] = {0.5, 0.5, 0.0, 0.0};
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

    glLineWidth(1.0);
    
    glBegin (GL_LINE_LOOP);
    for (int i=0; i<4; i++)
        glVertex2dv (self->footprint + 2*i);
    glEnd ();

    int draw_arrow = 0;
    if (draw_arrow) {
        double fp_length = self->footprint[0] - self->footprint[4];
        double fp_width = fabs (self->footprint[1] - self->footprint[3]);

        glPushMatrix ();
        glTranslatef (self->footprint[0] - fp_length/2,
                      self->footprint[1] - fp_width/2, 0);
        bot_gl_draw_arrow_2d (fp_length, fp_width, fp_length * 0.3, 
                              fp_width * 0.5, self->ehandler.hovering);
        glPopMatrix ();
    }
}


void
rrtstar_renderer_draw (BotViewer *viewer, BotRenderer *renderer) {
    
    RendererRRTStar *self = (RendererRRTStar *)renderer->user;

    glEnable (GL_LIGHTING);
    glDisable (GL_DEPTH_TEST);
    glEnable (GL_LINE_SMOOTH);
    glShadeModel (GL_SMOOTH);


    float color_curr[3];
    glLineWidth (2.0);

    color_curr[0] = 0.0;
    color_curr[1] = 1.0;
    color_curr[2] = 0.0;
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);

    //draw the new goal region
    glTranslatef(self->goal_mean.x, self->goal_mean.y, 0);
    bot_gl_draw_circle(self->goal_std);

    glBegin(GL_LINE_STRIP);  
    glVertex2f(0.0,0.0);
    
    glVertex2f(self->goal_std*cos(self->goal_theta),self->goal_std*sin(self->goal_theta));

    glEnd();

    glTranslatef(-self->goal_mean.x, -self->goal_mean.y, 0);

    glLineWidth (5.0);
    
    /*    
    // Draw the goal region
//     glColor3f (0.1, 1.0, 0.1);
    color_curr[0] = 0.1;
    color_curr[1] = 1.0;
    color_curr[2] = 0.1;
    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
    glLineWidth (5.0);
    glBegin (GL_LINE_LOOP);
    glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] - self->region_goal.size[1]/2.0, 0.0);
    glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] - self->region_goal.size[1]/2.0, 0.0);
    glVertex3f (self->region_goal.center[0] + self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] + self->region_goal.size[1]/2.0, 0.0);
    glVertex3f (self->region_goal.center[0] - self->region_goal.size[0]/2.0, 
                self->region_goal.center[1] + self->region_goal.size[1]/2.0, 0.0);
    glEnd ();
    */

    g_mutex_lock (self->mutex);

    
    
    /*glPopMatrix();
    
    double t_pos[3] = {.0,.0,.0};
    bot_gl_draw_text(t_pos, GLUT_BITMAP_HELVETICA_12, "Test", BOT_GL_DRAW_TEXT_DROP_SHADOW);
    */

    // Render the footprint if this and the render_traj options are enabled
    if ( (self->render_footprint) && (self->traj) && (self->render_traj)) {

        for (int i=0; i < self->traj->num_states; i++) {

            glPushMatrix();
            
            // compute the transformation to world coordinates
            BotTrans body_to_local;
            double rpy[] = {0,0,self->traj->states[i].t}; 
            body_to_local.trans_vec[0] = self->traj->states[i].x;
            body_to_local.trans_vec[1] = self->traj->states[i].y;
            body_to_local.trans_vec[2] = 0;
            bot_roll_pitch_yaw_to_quat (rpy, body_to_local.rot_quat);

            double body_quat_m[16];
            bot_trans_get_mat_4x4(&body_to_local, body_quat_m);

            double body_quat_m_opengl[16];
            bot_matrix_transpose_4x4d (body_quat_m, body_quat_m_opengl);

            glMultMatrixd (body_quat_m_opengl);

            draw_footprint (self);

            glPopMatrix();
        }    
    }
    
    
    if ( (self->render_tree) && (self->num_trees > 0) ) {
    
        // Draw all the nodes    
        int current_index = (self->index_trees - self->slider_history_no);
        while (current_index < 0)
            current_index += MAX_HISTORY;
        ripl_rrt_tree_t *tree = self->trees[current_index];
        glLineWidth (1.0);
//         glColor3f (0.3,1.0,0.3);
        color_curr[0] = 0.3;
        color_curr[1] = 1.0;
        color_curr[2] = 0.3;
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
        glBegin (GL_POINTS);    
        for (int i = 0; i < tree->num_nodes; i++) {
            glVertex3f (tree->nodes[i].state.x, tree->nodes[i].state.y, 0.0);
        }
        glEnd ();
    
    
        // Draw the edges
        //         glColor3f (1.0,0.3,0.3);
        for (int i = 0; i < tree->num_edges; i++) {
            glLineWidth (0.3);
            color_curr[0] = 1,0;
            color_curr[1] = 0.3;
            color_curr[2] = 0.3;
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
            glBegin (GL_LINE_STRIP);
            glVertex3f (tree->nodes[tree->edges[i][1]].state.x, 
                        tree->nodes[tree->edges[i][1]].state.y,
                        0.0);
            for (int j = 0; j < tree->traj_from_parent[tree->edges[i][0]].num_states; j++) {
                glVertex3f (tree->traj_from_parent[tree->edges[i][0]].states[j].x, 
                            tree->traj_from_parent[tree->edges[i][0]].states[j].y,
                            0.0);
            }
            glVertex3f (tree->nodes[tree->edges[i][0]].state.x, 
                        tree->nodes[tree->edges[i][0]].state.y,
                        0.0);
            glEnd();

        /*if (1) {
          char buf[128];
          double cost = 100; 
          sprintf(buf, "%5.1f", tree->nodes[tree->edges[i][0]].distance_from_root);
          double pos[3] = { tree->nodes[tree->edges[i][0]].state.x, 
                tree->nodes[tree->edges[i][0]].state.y, 0 };
          glColor3f(0.0,1.0,0);
          //                glutil_draw_text(pos, NULL, buf, GLUTIL_DRAW_TEXT_DROP_SHADOW);
          bot_gl_draw_text(pos, NULL, buf, 0);
        }*/
        }
        
        int *edge_counts = (int *) calloc(tree->num_nodes,sizeof(int)); 

        for (int i = 0; i < tree->num_edges; i++) {
            glLineWidth (0.3);
            color_curr[0] = 1.0;
            color_curr[1] = 0.3;
            color_curr[2] = 0.3;
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
            glBegin (GL_LINE_STRIP);

            edge_counts[tree->edges[i][0]]++;
            edge_counts[tree->edges[i][1]]++;
            
            glVertex3f (tree->nodes[tree->edges[i][1]].state.x, 
                        tree->nodes[tree->edges[i][1]].state.y,
                        0.0);

            for (int j = 0; j < tree->traj_from_parent[tree->edges[i][0]].num_states; j++) {
                glVertex3f (tree->traj_from_parent[tree->edges[i][0]].states[j].x, 
                            tree->traj_from_parent[tree->edges[i][0]].states[j].y,
                            0.0);
                
            }
            glVertex3f (tree->nodes[tree->edges[i][0]].state.x, 
                        tree->nodes[tree->edges[i][0]].state.y,
                        0.0);
            
            glEnd();
        }
        
        // Render the cost
        if (self->render_costs) {
            for(int i=0; i < tree->num_nodes; i++){
                if (edge_counts[i] ==1) {
                    char buf[128];
                    double cost = 100; 
                    sprintf(buf, "%5.1f", tree->nodes[i].distance_from_root);
                    double pos[3] = { tree->nodes[i].state.x, 
                                      tree->nodes[i].state.y, 0 };
                    color_curr[0] = 0.0;
                    color_curr[1] = 1.0;
                    color_curr[2] = 0.0;
                    glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
                    //glColor3f (1.0,1.0,0.0);
                    bot_gl_draw_text(pos, NULL, buf, 0);
                }
            }
        }
        
        free (edge_counts);
    }
    

    // Now render the best trajectory
    if ( (self->render_traj) && (self->traj) ) {
        // Draw the edges in the trajectory
        color_curr[0] = 0.8;
        color_curr[1] = 1.0;
        color_curr[2] = 0.8;
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_curr);
        glLineWidth (3.0);
        glBegin (GL_LINE_STRIP);
        for (int i = 0; i < self->traj->num_states; i++)
            glVertex3f (self->traj->states[i].x,
                        self->traj->states[i].y,
                        0.0);
        glEnd();
    }


    
    glDisable (GL_STENCIL_TEST);
    glDisable (GL_LIGHTING);
    //glDisable (GL_DEPTH_TEST);
    glDisable (GL_LINE_SMOOTH);
    glEnable (GL_DEPTH_TEST);

    glFlush ();
    
    g_mutex_unlock (self->mutex);

}


void 
setup_renderer_rrtstar (BotViewer *viewer, int priority, lcm_t *_lcm) {

    RendererRRTStar *self = (RendererRRTStar *) malloc (sizeof (RendererRRTStar));
    
    self->num_trees = 0;
    
    self->index_trees = 0;
    for (int i = 0; i < MAX_HISTORY; i++)
        self->trees[i] = NULL;

    self->traj = NULL;
    
    self->mutex = g_mutex_new ();
    
    self->mouseNodeXY[0] = 0.0;
    self->mouseNodeXY[1] = 0.0;

    BotRenderer *renderer = &(self->renderer);

    renderer->draw = rrtstar_renderer_draw;
    renderer->destroy = rrtstar_renderer_destroy;
    renderer->name = RENDERER_NAME;
    renderer->widget = gtk_vbox_new (FALSE, 0);
    renderer->enabled = 1;
    renderer->user = self;

    BotEventHandler *ehandler = &(self->ehandler);
    memset(ehandler, 0x00, sizeof(BotEventHandler));
    ehandler->name = RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_motion = mouse_motion;
    ehandler->mouse_release  = mouse_release;
    ehandler->mouse_scroll = NULL;
    ehandler->pick_query = NULL;
    ehandler->hover_query = NULL;
    ehandler->key_press = key_press;
    ehandler->user = self;

    self->lcm = _lcm; 
    self->viewer = viewer;
    self->pw = BOT_GTK_PARAM_WIDGET (bot_gtk_param_widget_new());
    
    self->key_goal = 0;
    self->region_goal.center[0] = 8.0;
    self->region_goal.center[1] = 8.0;
    self->region_goal.size[0] = 3.0;
    self->region_goal.size[1] = 3.0;
    self->last_region_goal.center[0] = self->region_goal.center[0];
    self->last_region_goal.center[1] = self->region_goal.center[1];
    self->last_region_goal.size[0] = self->region_goal.size[0];
    self->last_region_goal.size[1] = self->region_goal.size[1];

    self->bot_pose_last = NULL;

    // Add the history slider
    gtk_box_pack_start (GTK_BOX(renderer->widget), GTK_WIDGET(self->pw), TRUE, TRUE, 0);
    bot_gtk_param_widget_add_int (self->pw, SLIDER_HISTORY, 
                                  BOT_GTK_PARAM_WIDGET_SLIDER, 1, MAX_HISTORY, 1, 1);
    bot_gtk_param_widget_set_enabled (self->pw, SLIDER_HISTORY, 0);
    self->slider_history_no = 1;
    
    // Add render traj check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_RENDER_TRAJ, 0, NULL);
    self->render_traj = FALSE;

    // Add render tree check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_RENDER_TREE, 1, NULL);
    self->render_tree = TRUE;

    // Add render costs check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_RENDER_COSTS, 0, NULL);
    self->render_costs = FALSE;


    
    // Add render footprint check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_RENDER_FOOTPRINT, 1, NULL);

    // Add render ignore heading check box
    bot_gtk_param_widget_add_booleans (self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX,
                                       CHECKBOX_DO_HEADING, 1, NULL);

    self->render_footprint = TRUE;    
    self->do_heading = TRUE;

    // Add the clear history button
    GtkWidget *clear_button = gtk_button_new_with_label ("Clear History");
    gtk_box_pack_start(GTK_BOX(renderer->widget), clear_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(clear_button), "clicked",
                     G_CALLBACK(on_clear_button), self);
    
    // Add the start  button
    GtkWidget *start_button = gtk_button_new_with_label ("Start RRT");
    gtk_box_pack_start(GTK_BOX(renderer->widget), start_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(start_button), "clicked",
                     G_CALLBACK(on_start_button), self);

    // Add the modify goal box button
    GtkWidget *modify_goal_button = gtk_button_new_with_label ("Modify Goal Area");
    gtk_box_pack_start(GTK_BOX(renderer->widget), modify_goal_button,
                       FALSE, FALSE, 0);
    g_signal_connect(G_OBJECT(modify_goal_button), "clicked",
                     G_CALLBACK(on_modify_goal_button), self);
    self->modify_goal_button = modify_goal_button;

    // Show the widgets
    gtk_widget_show_all (renderer->widget);
    g_signal_connect (G_OBJECT (self->pw), "changed", G_CALLBACK (on_param_widget_changed), self);
    
    // Subscribe to MP_RRT_TREE and MP_RRT_TRAJ message
    ripl_rrt_tree_t_subscribe (self->lcm, "RRTSTAR_TREE", on_mp_rrt_tree, self);
    ripl_rrt_traj_t_subscribe (self->lcm, "RRTSTAR_TRAJECTORY", on_mp_rrt_traj, self);
    ripl_rrt_command_t_subscribe (self->lcm, "RRTSTAR_COMMAND", on_mp_rrt_cmd, self);

    // Subscribe to the POSE message
    bot_core_pose_t_subscribe (self->lcm, "POSE", on_bot_pose, self);
    
    bot_viewer_add_renderer (viewer, renderer, priority);
    bot_viewer_add_event_handler(viewer, ehandler, priority);

    return;
}
