
#define STOP_DIST 0.5 //3.0     //(m)

#define TV_STOPPED_THRESHOLD 0.05
#define RV_STOPPED_THRESHOLD 0.05

// The maximum allowable obs_max cost along the
// committed trajectory above which the bot will stop
#define COMMIT_OBS_MAX_COLLISION 200

// These should be moved to config
#define BOT_AT_GOAL_DIST 1.0
#define BOT_AT_GOAL_TIGHT_DIST 1.0
#define BOT_NEAR_GOAL_DIST 4.0 //8.0

#ifdef __APPLE
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>
#include <sys/time.h>
#include <getopt.h>
#include <string.h>
#include <unistd.h>

#include <lcm/lcm.h>
#include <bot_core/bot_core.h>

#include <lcmtypes/bot_core.h>
#include <lcmtypes/rrtstar.h>
#include <lcmtypes/hr_lcmtypes.h>
#include <bot_lcmgl_client/lcmgl.h>

#include "opttree.h"
#include <bot_param/param_client.h>
#include <geom_utils/convexhull.h>

#define DEFAULT_CHECK_GRIDMAP_WIDTH_BUFFER 0.05 //0.2

#define GETCFI(key, val, prefix)                        \
    if (bot_param_get_int (config, key, val) != 0) {    \
        DBG_W ("%s: Error reading %s!\n", prefix, key); \
        return -1;                                      \
    }

#define GETCFD(key, val, prefix)                        \
    if (bot_param_get_int (config, key, val) != 0) {    \
        DBG_W ("%s: Error reading %s!\n", prefix, key); \
        return -1;                                      \
    }

#define GETCFB(key, val, prefix)                        \
    if (bot_param_get_int (config, key, val) != 0) {    \
        DBG_W ("%s: Error reading %s!\n", prefix, key); \
        return -1;                                      \
    }

typedef struct _rrt_config_t {
    int iteration_limit;       // Limit on the number of iterations that the algorithm is run for
    int time_limit;            // Limit on the time that the algorithm run for

    // Tree publish interval
    int tree_pub_iteration_limit;    
    int tree_pub_time_limit;
    gboolean tree_pub_final;

    // Iteration and time interval for publishing the best trajectory
    int traj_pub_iteration_limit;
    int traj_pub_time_limit;
    gboolean traj_pub_final;

    double commit_time;
} rrt_config_t; 


typedef struct _rrtstar_t {
    lcm_t *lcm;
    BotParam * param;
    BotFrames *frames;
    rrt_config_t config; 
    opttree_t *opttree;
    
    erlcm_rrt_environment_t *rrt_environment_last;
    
    GMutex *mutex;
    GMutex *plan_mutex;
    GMutex *running_mutex;
    
    GMutex *stop_iter_mutex;
    
    GMutex *at_goal_mutex;
    
    GMainLoop *mainloop; 

    int is_new_goal;
    
    gboolean trash_tree_on_wp;
    gboolean verbose_screen;
    gboolean verbose_motion;
    gboolean verbose_tree_msg;
    gboolean verbose_traj_msg;
    gboolean continuous_map_updates;
    gboolean enable_turn_in_place;
    gboolean perform_lazy_collision_check;
    gboolean perform_branch_and_bound;
    int basic_failsafe;
    
    int num_nodes;
    double default_tv;

    double lazy_collision_check_distance;


    double lastx;
    double lasty;
    
    int iteration_no;
    int64_t time_start;
    
    int stop_iter;
    int is_running; 
    
    region_2d_t operating_region; 
    

    GSList *committed_traj;

    double commited_point[3];
    
    bot_core_pose_t *bot_pose_last;
    
    GThread *planning_thread;
    
    gboolean executing_mp_cmd;    
    
    erlcm_goal_list_t *goal_list;

    erlcm_goal_list_t *goal_list_global;

    int goal_type; //0 -normal goal // 1 - elevator/door 
    int current_goal_ind; 
    int do_turn_only; 
    
    erlcm_velocity_msg_t *vel_status; 
    int bot_at_end_of_committed; 
    bot_lcmgl_t *lcmgl_goal; 
    bot_lcmgl_t *lcmgl_operating_region; 
    bot_lcmgl_t *lcmgl_collision;

    gboolean sent_at_goal; 
    gboolean commited_to_final_goal; 
    
    bot_lcmgl_t *lcmgl_committed_point; 

    int32_t goal_id; 
    erlcm_rrt_goal_status_t *goal_status; 

    int publish_waypoint_status; 
    
} rrtstar_t;


void draw_goal(rrtstar_t *self){
    if(self->lcmgl_goal && self->goal_list){
        bot_lcmgl_t *lcmgl = self->lcmgl_goal; 
        //lcmglLineSize(8);
        bot_lcmgl_line_width(lcmgl, 5);
        lcmglColor3f(1.0, 0.0, 0.0);
        //ToDo: add heading 
        int c_ind = self->current_goal_ind; 
        
        double goal[3] = {self->goal_list->goals[c_ind].pos[0], self->goal_list->goals[c_ind].pos[1], 0.05}; 
        lcmglCircle(goal, 0.6);
        bot_lcmgl_switch_buffer(self->lcmgl_goal);
    }  
}

void get_commited_end_point(rrtstar_t *self){
    GSList *state_ptr = self->committed_traj;
    state_t *curr_state = (state_t *)(state_ptr->data);      
    
    while (g_slist_next(state_ptr)) {
        curr_state = (state_t *)(state_ptr->data);
        state_ptr = g_slist_next (state_ptr);    
    }
    if(curr_state){
        self->commited_point[0] = curr_state->x[0];
        self->commited_point[1] = curr_state->x[1];
        self->commited_point[1] = curr_state->x[2];
    }    
}

void draw_committed_point(rrtstar_t *self, state_t *latest_goal){
    if(self->lcmgl_committed_point && self->goal_list){
      
        bot_lcmgl_t *lcmgl = self->lcmgl_committed_point;
        lcmglPointSize(6);
        lcmglColor3f(0.0, 0.0, 1.0);
        //ToDo: add heading 
        GSList *state_ptr = self->committed_traj;
        state_t *curr_state = (state_t *)(state_ptr->data);      
    
        while (g_slist_next(state_ptr)) {
            curr_state = (state_t *)(state_ptr->data);
            double goal[3] = {curr_state->x[0], curr_state->x[1], 0.05}; 
            lcmglCircle(goal, 0.3);
            state_ptr = g_slist_next (state_ptr);    
        }
      
        if(curr_state){
            double goal[3] = {curr_state->x[0], curr_state->x[1], 0}; 
            lcmglColor3f(1.0, .0,0.0);
            lcmglCircle(goal, 0.3);
        }

        bot_lcmgl_switch_buffer(self->lcmgl_committed_point);
    }  
}

int 
is_bot_at_end_of_committed_traj (rrtstar_t *self);

int get_bot_rel_quad (double x, double y, double x_0, double y_0, double yaw )
{
    double theta_k = atan2(y - y_0, x - x_0);
    theta_k -= yaw;
    while (theta_k <= 0)
        theta_k += 2*M_PI;
    if ( (0 <= theta_k) && (theta_k <= M_PI/2.0) )
        return 1;
    if ( (M_PI/2.0 <= theta_k) && (theta_k <= M_PI) )
        return 2; //behind
    if ( (M_PI <= theta_k) && (theta_k <= 3*M_PI/2.0) )
        return 3; //behind
    if ( (3.0*M_PI/2.0 <= theta_k) && (theta_k <= 2.0*M_PI) )
        return 4;
    return -1;
}

int 
is_bot_at_goal (rrtstar_t *self, erlcm_goal_t *latest_goal) {  
    if (!self->bot_pose_last)
        return -1;
    
    if(!latest_goal){
        return -1;
    }
    
    double dist_x = self->bot_pose_last->pos[0] - latest_goal->pos[0];
    double dist_y = self->bot_pose_last->pos[1] - latest_goal->pos[1];
    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
    
    if (dist < BOT_AT_GOAL_DIST){
        if(self->verbose_motion)
            fprintf(stderr,"Already at goal - ignoring\n");
    
        return 1;
    }
    return 0;
}

int 
is_bot_at_goal_tight (rrtstar_t *self, erlcm_goal_t *latest_goal) {  
    if (!self->bot_pose_last)
        return -1;
    
    if(!latest_goal){
        return -1;
    }
    
    double dist_x = self->bot_pose_last->pos[0] - latest_goal->pos[0];
    double dist_y = self->bot_pose_last->pos[1] - latest_goal->pos[1];
    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
    
    if(dist < BOT_AT_GOAL_TIGHT_DIST){
        //fprintf(stderr, "Dist to goal : %f\n", dist); 
    }

    if (dist < 0.4){
        if(self->verbose_motion)
            fprintf(stderr,"Already at goal - ignoring\n");
    
        return 1;
    }
    return 0;
}

int 
is_bot_near_goal (rrtstar_t *self, erlcm_goal_t *latest_goal) {  
    if (!self->bot_pose_last)
        return -1;
    if(!latest_goal){
        return -1;
    }
    
    double dist_x = self->bot_pose_last->pos[0] - latest_goal->pos[0];
    double dist_y = self->bot_pose_last->pos[1] - latest_goal->pos[1];
    double dist = sqrt(dist_x * dist_x + dist_y * dist_y);
    
    if (dist < BOT_NEAR_GOAL_DIST){
        if(self->verbose_motion)
            fprintf(stdout,"Bot is near goal\n");
        return 1;
    }
    return 0;
}

int 
is_robot_moving( rrtstar_t *self){
    if(self->vel_status !=NULL){
        if(self->verbose_motion)
            fprintf(stdout,"TV :%f RV :%f\n", self->vel_status->tv, self->vel_status->rv);
        if(fabs(self->vel_status->tv) < TV_STOPPED_THRESHOLD &&
           fabs(self->vel_status->rv) < RV_STOPPED_THRESHOLD)
            return 0;
        else
            return 1;
    }
    else
        return 0;
}

int
is_committed_trajectory_in_collision (rrtstar_t *self) {

    //commited traj in collision check has been disabled for now - Matt (to fix)
    return 0;

    // The path is in collision if either the maximum obs_max along the
    // path exceeds a threshold or a segment enters a -1 cost cell (unknown)

    double max_obs_max = 0;
    int pass_thru_unknown = 0;

    if (!self->committed_traj){
        fprintf (stdout, "Checking committed trajectory for collision: No committed trajectory!\n");
        return 0;
    }

    check_gridmap_update (self->opttree->optsys->grid);

    GSList *state_i_ptr = self->committed_traj;
    GSList *state_f_ptr = g_slist_next (state_i_ptr);

    while (state_f_ptr) {
        state_t *state_start = (state_t *) state_i_ptr->data;
        state_t *state_next = (state_t *) state_f_ptr->data;

        double dist = sqrt(bot_sq(state_start->x[0]-state_next->x[0]) + bot_sq(state_start->x[1]-state_next->x[1]));
        struct check_path_result path_res;
        int is_forward = 1;
        int failsafe = 0; //2; // When failsafe >= 1, reduce vehicle footprint (width)
        check_gridmap_check_path (self->opttree->optsys->grid, is_forward, failsafe,
                                  state_start->x[0], state_start->x[1], state_start->x[2],
                                  state_next->x[0], state_next->x[1], state_next->x[2],
                                  &path_res);

        //fprintf (stdout, "path_res.obs_max = %.2f\n", path_res.obs_max);
        if (path_res.obs_max > max_obs_max)
            max_obs_max = path_res.obs_max;
        else if (path_res.obs_max < 0)
            pass_thru_unknown = 1;

        //maybe we should render the robot footprint here 
        
        // Now get the next pair of states
        state_i_ptr = state_f_ptr;
        state_f_ptr = g_slist_next (state_i_ptr);
    }

    if (max_obs_max > COMMIT_OBS_MAX_COLLISION) {
        fprintf (stdout, "Committed trajectory is in collision. Max obs_max = %.4f!\n", max_obs_max);
        return 1;
    }
    else if (pass_thru_unknown) {
        fprintf (stdout, "Committed trajectory passes through an unknown or full cost region of the gridmap!\n");
        return 1;
    }
    else
        return 0;
}

static void
on_rrt_command (const lcm_recv_buf_t *buf, const char *channel,
                const erlcm_rrt_command_t *msg, void *user);


static void
on_goal_status (const lcm_recv_buf_t *buf, const char *channel,
                    const erlcm_rrt_goal_status_t *msg, void *user){
    rrtstar_t *self = (rrtstar_t*) user;
    if(self->goal_status !=NULL){
        erlcm_rrt_goal_status_t_destroy(self->goal_status);
    }
    self->goal_status = erlcm_rrt_goal_status_t_copy(msg);
}

static void
on_velocity_status (const lcm_recv_buf_t *buf, const char *channel,
                    const erlcm_velocity_msg_t *msg, void *user){
    rrtstar_t *self = (rrtstar_t*) user;
    if(self->vel_status !=NULL){
        erlcm_velocity_msg_t_destroy(self->vel_status);
    }
    self->vel_status = erlcm_velocity_msg_t_copy(msg);  
}


void stop_controller_motion(rrtstar_t *self){
    
    erlcm_ref_point_list_t pub = {
        .num_ref_points = 0,
        .ref_points = NULL,
        .mode = ERLCM_REF_POINT_LIST_T_NORMAL_MOTION,
        .id = -1//self->goal_id
    };
    erlcm_ref_point_list_t_publish (self->lcm, "GOAL_REF_LIST", &pub);
}

//to use this - we would need to change the id size to 64 bit 
uint64_t 
get_unique_id (void)
{
    return (0xefffffffffffffff&(bot_timestamp_now()<<8)) + 256*rand()/RAND_MAX;
}

uint32_t 
get_unique_id_32t (void)
{
    return (0xefffffff&(bot_timestamp_now()<<8)) + 256*rand()/RAND_MAX;
}

void estop_controller (rrtstar_t *self) {
    erlcm_speech_cmd_t msg = {
        .utime = bot_timestamp_now(),
        .cmd_type = "FOLLOWER",
        .cmd_property = "STOP",
    };
    erlcm_speech_cmd_t_publish (self->lcm, "WAYPOINT_NAVIGATOR", &msg);
}

static void convert_goal_list_to_global(rrtstar_t *self){
    if(!self->goal_list_global)
        return;

    erlcm_goal_list_t *goal_list = self->goal_list_global;

    BotTrans local_to_global;

    bot_frames_get_trans_with_utime(self->frames, "local", "global", goal_list->utime, &local_to_global);

    for(int i=0; i < goal_list->num_goals; i++){
        erlcm_goal_t *goal = &goal_list->goals[i];

        double pos_local[3] = {goal->pos[0], goal->pos[1], 0};
        double pos_global[3];
        bot_frames_transform_vec (self->frames, "local", "global",  pos_local, pos_global);
        
        double rpy_local[3] = {0, 0, goal->theta};
        double quat_local[4];
        bot_roll_pitch_yaw_to_quat(rpy_local, quat_local);
        
        double quat_global[4];
        bot_quat_mult (quat_global, local_to_global.rot_quat, quat_local);

        double rpy_global[3];
        bot_quat_to_roll_pitch_yaw(quat_global, rpy_global);

        goal->pos[0] = pos_global[0];
        goal->pos[1] = pos_global[1];

        goal->theta = rpy_global[2];
    }    
}

static void update_local_goal_list(rrtstar_t *self){
    if(0){
        //don't update 
        return;
    }
    if(!self->goal_list_global ||  !self->goal_list)
        return;

    erlcm_goal_list_t *g_list = self->goal_list_global;
    erlcm_goal_list_t *l_list = self->goal_list;

    if(l_list->num_goals != g_list->num_goals){
        fprintf(stderr, "Goal list sizes are different\n");
        exit(-1);
    }

    BotTrans global_to_local;

    bot_frames_get_trans(self->frames, "global", "local", &global_to_local);

    for(int i=0; i < g_list->num_goals; i++){
        erlcm_goal_t *goal = &g_list->goals[i];
        erlcm_goal_t *l_goal = &l_list->goals[i];

        double pos_global[3] = {goal->pos[0], goal->pos[1], 0};
        double pos_local[3];
        bot_frames_transform_vec (self->frames, "global", "local",  pos_global, pos_local);
        
        double rpy_global[3] = {0, 0, goal->theta};
        double quat_global[4];
        bot_roll_pitch_yaw_to_quat(rpy_global, quat_global);
        
        double quat_local[4];
        bot_quat_mult (quat_local, global_to_local.rot_quat, quat_global);

        double rpy_local[3];
        bot_quat_to_roll_pitch_yaw(quat_local, rpy_local);

        l_goal->pos[0] = pos_local[0];
        l_goal->pos[1] = pos_local[1];

        l_goal->theta = rpy_local[2];
    }    
}

// If we get a new goal list, we currently stop the robot and stop searching.
// We should instead adapt the tree according to the new goal (i.e., identify 
// the (new) optimial path to the goal if it exists and continue planning). 
// Alternatively, the simplist thing to do would be to trash the tree after the 
// committed trajectory and restart search while the vehicle drives to the committed
// trajectory
static void
on_goals(const lcm_recv_buf_t * rbuf, const char *channel,     
         const erlcm_goal_list_t *msg, void *user) {
    rrtstar_t *self = (rrtstar_t*) user;    

    int is_running = 0; 
    //wait while its cleared 
    g_mutex_lock(self->running_mutex);
    is_running = self->is_running;
    g_mutex_unlock(self->running_mutex);
    
    if (self->goal_list){
        erlcm_goal_list_t_destroy(self->goal_list);   
    }    
    
    if(self->goal_list_global){
        erlcm_goal_list_t_destroy(self->goal_list_global);   
    }

    // We probably don't want to be stopping the robot and rrtstar
    if(is_running){
        g_mutex_lock (self->stop_iter_mutex);
        self->stop_iter = 1;
        g_mutex_unlock (self->stop_iter_mutex); 
        stop_controller_motion(self);
    }
    
    self->goal_list = erlcm_goal_list_t_copy(msg);
    self->goal_list_global = erlcm_goal_list_t_copy(msg);

    convert_goal_list_to_global(self);

    if (!strcmp(channel, "RRTSTAR_ELEVATOR_GOALS")) {
        fprintf(stderr, "New elevator goal received - From Sender ID : %d\n", 
                (int)msg->sender_id);
        self->goal_type = 1;        
    }
    else if(!strcmp(channel, "RRTSTAR_GOALS")) {
        fprintf(stderr, "New standard goal received - From Sender ID : %d\n", 
                (int)msg->sender_id);
        self->goal_type = 0;        
    }
    //we should send this to the waypoint follower 
    update_local_goal_list(self);
    erlcm_goal_t *latest_goal = &(self->goal_list->goals[self->goal_list->num_goals-1]);
    
    self->sent_at_goal = FALSE;
    self->commited_to_final_goal = FALSE;

    if(self->goal_list->goals[0].do_turn_only){
        fprintf(stderr, "-------- Turn-in-place goal received\n");
    }

    //ignore the goal if the robot is near enough to the goal 
    if (is_bot_at_goal(self,latest_goal) && !self->goal_list->goals[0].do_turn_only) {
        //we are already at the goal 

        fprintf(stderr, "+++++ We are at the goal - not checking for orientation\n");

        erlcm_speech_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.cmd_type = "WAYPOINT_STATUS"; 
        msg.cmd_property = "REACHED"; 
        erlcm_speech_cmd_t_publish (self->lcm, "WAYPOINT_STATUS", &msg);

        return;
    }
    
    if(self->verbose_motion)
        fprintf(stdout,"=======New goal set received=======\n");

    //which waypoint do we pick at the start 
    self->current_goal_ind = 0; 
    
    self->is_new_goal =1;
  
    g_mutex_unlock (self->plan_mutex);
    
}

double get_max(double *val, int size){
    double max = -100000;
    for(int i=0;i<size;i++){
        if(max < val[i]){
            max = val[i];
        }
    }  
    return max; 
}


double get_min(double *val, int size){
    double min = 100000;
    for(int i=0;i<size;i++){
        if(min > val[i]){
            min = val[i];
        }
    }  
    return min; 
}

// check to see if the robot is at the end of the trajectory it's currently committed to
int 
check_bot_at_end_of_committed_traj (rrtstar_t *self, state_t *state_end, 
                                    double threshold, double *dist) {
  
    if (!self->bot_pose_last){
        fprintf(stderr,"Error - No last pose");
        return -1;
    }
       
    double dist_x = self->bot_pose_last->pos[0] - state_end->x[0];
    double dist_y = self->bot_pose_last->pos[1] - state_end->x[1];
    *dist = sqrt(dist_x * dist_x + dist_y * dist_y);    

    if (*dist < threshold){
        return 1;
    }

    return 0;    
}

// lcm callback function for pose information
static void
on_bot_pose (const lcm_recv_buf_t *rbuf, const char *channel,
             const bot_core_pose_t *msg, void *user)
{
    rrtstar_t *self = (rrtstar_t*) user;

    if (self->bot_pose_last)
        bot_core_pose_t_destroy (self->bot_pose_last);
    
    self->bot_pose_last = bot_core_pose_t_copy (msg);

    //adding a check here for end of committed traj

    int new_traj_segment = 0;

    static double committed_end[2] = {.0,.0};
    if(self->committed_traj){
        GSList *state_ptr = self->committed_traj;
        while (g_slist_next(state_ptr)) {
            state_ptr = g_slist_next (state_ptr);
        }
        state_t *state_end = (state_t *)(state_ptr->data);
        
        draw_committed_point(self, state_end);
        
        int bot_at_end_of_committed = 0;
        
        g_mutex_lock (self->at_goal_mutex);
        bot_at_end_of_committed = self->bot_at_end_of_committed;          
        g_mutex_unlock (self->at_goal_mutex);
        
        if(self->verbose_motion){
            if(committed_end[0] != state_end->x[0] || committed_end[1] != state_end->x[1]){
                fprintf(stdout, "+++++++ New Traj Segment ++++++++ (%.2f,%.2f) => (%.2f, %.2f) =: At goal : %d\n", 
                        committed_end[0], committed_end[1], state_end->x[0], state_end->x[1], bot_at_end_of_committed);
            }
        }
        
        
        if((committed_end[0] != state_end->x[0] || committed_end[1] != state_end->x[1]) || !bot_at_end_of_committed){
            new_traj_segment = 1;
            
            committed_end[0] = state_end->x[0];
            committed_end[1] = state_end->x[1];

            int is_now_at_committed = 0;
        
            double dist = 0.0;
            
            is_now_at_committed = check_bot_at_end_of_committed_traj(self, state_end, 1.0, &dist);
            g_mutex_lock (self->at_goal_mutex);
    
            if(self->bot_at_end_of_committed ==0 && is_now_at_committed){
                fprintf(stdout,"++++++++++ Arrived at committed traj ++++++++++\n");                
            }     
            self->bot_at_end_of_committed = is_now_at_committed; 
            g_mutex_unlock (self->at_goal_mutex);

        }

        if(self->commited_to_final_goal == TRUE){
            double dist = 0.0;
            int is_now_at_goal = check_bot_at_end_of_committed_traj(self, state_end, 0.4, &dist);
            /*if(dist < 1.0){
                fprintf(stderr, "Checking at goal : %.2f \n", dist);
                }*/
            /*if(is_now_at_goal){
                fprintf(stderr, " +++++++ Send bot at goal - and clear\n");
                self->commited_to_final_goal = FALSE;      
                //for now send the basic at waypoint speech message 
                erlcm_speech_cmd_t msg;
                msg.utime = bot_timestamp_now();
                msg.cmd_type = "WAYPOINT_STATUS"; 
                msg.cmd_property = "REACHED"; 
                erlcm_speech_cmd_t_publish (self->lcm, "WAYPOINT_STATUS", &msg);
                }*/            
        }        
    }
    
    /*if(self->goal_list){
        erlcm_goal_t *final_goal = &(self->goal_list->goals[self->goal_list->num_goals-1]);        
        int bot_at_goal = is_bot_at_goal_tight(self,final_goal);
        if(bot_at_goal){
            fprintf(stderr, "Bot at goal \n" );
        }
        if(bot_at_goal && !self->sent_at_goal){
            fprintf(stderr, " +++++++ Send bot at goal - and clear\n");
            self->sent_at_goal = TRUE; 
        }
        }*/

    /*double bot_rpy[3];
    bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, bot_rpy);

    fprintf(stderr, "Pose : %f,%f,%f\n", 
            self->bot_pose_last->pos[0], 
            self->bot_pose_last->pos[1], 
            bot_rpy[2]);*/    

    return;
}

void reset_bot_at_committed_traj (rrtstar_t *self) {
    g_mutex_lock (self->at_goal_mutex);
    self->bot_at_end_of_committed = 0;     
    g_mutex_unlock (self->at_goal_mutex);
}

int 
is_bot_at_end_of_committed_traj (rrtstar_t *self) {
    g_mutex_lock (self->at_goal_mutex);
    int at_goal = self->bot_at_end_of_committed;    
      
    g_mutex_unlock (self->at_goal_mutex);
    
    return at_goal; 
}

// initialization 

void read_parameters(rrtstar_t *self)
{
    self->config.iteration_limit =  bot_param_get_int_or_fail (self->param,"motion_planner.rrtstar.iteration_limit");
    self->config.time_limit =  bot_param_get_int_or_fail (self->param,"motion_planner.rrtstar.time_limit");
    self->config.tree_pub_iteration_limit =  bot_param_get_int_or_fail (self->param,"motion_planner.rrtstar.tree_pub_iteration_limit");
    self->config.tree_pub_time_limit =  bot_param_get_int_or_fail (self->param,"motion_planner.rrtstar.tree_pub_time_limit");
    self->config.tree_pub_final =  bot_param_get_boolean_or_fail (self->param,"motion_planner.rrtstar.tree_pub_final");
    self->config.traj_pub_iteration_limit =  bot_param_get_int_or_fail (self->param,"motion_planner.rrtstar.traj_pub_iteration_limit");
    self->config.traj_pub_time_limit =  bot_param_get_int_or_fail (self->param,"motion_planner.rrtstar.traj_pub_time_limit");
    self->config.traj_pub_final =  bot_param_get_boolean_or_fail (self->param,"motion_planner.rrtstar.traj_pub_final");
    self->default_tv = bot_param_get_double_or_fail (self->param, "motion_planner.speed_design.default_tv");
    self->lazy_collision_check_distance = bot_param_get_double_or_fail (self->param,"motion_planner.rrtstar.lazy_collision_check_distance");
    self->config.commit_time =  bot_param_get_double_or_fail (self->param,"motion_planner.rrtstar.commit_time");
}


rrtstar_t *rrtstar_create(gboolean sensing_only_local, gboolean trash_tree_on_wp, gboolean verbose, gboolean draw, 
                          gboolean clear_using_laser, gboolean sensing_only_small, double check_gridmap_width_buffer) {
    
    rrtstar_t *self = (rrtstar_t *) calloc (1, sizeof(rrtstar_t));
    g_thread_init(NULL);
    self->rrt_environment_last = NULL;
    self->lcm = bot_lcm_get_global (NULL);    
    if (!self->lcm) {
        fprintf (stderr, "Unable to get LCM instance\n");
        return NULL;
    }
    self->param = bot_param_new_from_server(self->lcm, 1);

    if (!self->param) {
      fprintf (stderr, "Unable to get BotParam instance\n");
      return NULL;
    }

    self->frames = bot_frames_get_global (self->lcm, self->param);
    self->lcmgl_goal = bot_lcmgl_init (self->lcm, "RRT_STAR_CURRENT_GOAL");
    self->lcmgl_committed_point = bot_lcmgl_init (self->lcm, "RRT_STAR_COMMITTED_POINT");
    self->lcmgl_collision = bot_lcmgl_init (self->lcm, "RRT_STAR_COLLISION");
    self->lcmgl_operating_region = bot_lcmgl_init (self->lcm, "RRT_STAR_OR");
    //read params from config file 
    read_parameters(self);    

    stop_controller_motion(self);
    
    self->verbose_screen = FALSE;
    self->verbose_tree_msg = TRUE;
    self->verbose_traj_msg = TRUE;
    self->trash_tree_on_wp = trash_tree_on_wp;
    self->sent_at_goal = FALSE; 
    self->commited_to_final_goal = FALSE;
    
    self->goal_id = 0; 
    self->goal_status = NULL;

    if(self->trash_tree_on_wp){
        fprintf(stderr, "Trashing Tree \n");
    }

    self->verbose_motion = verbose;

    self->mutex = g_mutex_new ();
    self->plan_mutex = g_mutex_new ();
    self->running_mutex = g_mutex_new ();
    self->stop_iter_mutex = g_mutex_new ();
    self->at_goal_mutex = g_mutex_new ();

    self->committed_traj = NULL;
    self->bot_pose_last = NULL;
    self->goal_list = NULL;
    self->goal_list_global = NULL;
    
    self->opttree = opttree_create (sensing_only_local, draw, clear_using_laser, sensing_only_small, check_gridmap_width_buffer);
    
    fprintf(stderr, "Done creating\n");
    self->opttree->optsys->failsafe_level = self->basic_failsafe;

    //pausing the map
    if(!self->continuous_map_updates){
        check_gridmap_off(self->opttree->optsys->grid);
    }
    else{
        check_gridmap_on(self->opttree->optsys->grid);
    }

    self->executing_mp_cmd = FALSE;
    
    erlcm_goal_list_t_subscribe(self->lcm, "RRTSTAR_GOALS", on_goals, self);

    erlcm_goal_list_t_subscribe(self->lcm, "RRTSTAR_ELEVATOR_GOALS", on_goals, self);

    erlcm_velocity_msg_t_subscribe(self->lcm, "ROBOT_VELOCITY_STATUS", on_velocity_status, self);
    erlcm_rrt_goal_status_t_subscribe(self->lcm, "TRAJECTORY_CONTROLLER_GOAL_STATUS", 
                                    on_goal_status, self); 
    
    bot_core_pose_t_subscribe (self->lcm, "POSE", on_bot_pose, self);
 
    return self;
}

//publishes optimal path to lcm, this is read by the controller
int 
optmain_publish_optimal_path (rrtstar_t *self, gboolean rampup_speed, 
                              double *new_root, int invalid_root) {
  
    int count = 0;
    erlcm_ref_point_t goal;

    int commited_node_id = 0;
    int found_commited_root = 0;
        
    int n_states = 0; 

    GSList *optstates_list = NULL;
    node_t *node_curr = self->opttree->lower_bound_node;
    while (node_curr) {
        optstates_list = g_slist_prepend (optstates_list, node_curr);
        n_states += g_slist_length (node_curr->traj_from_parent) + 1;
        node_curr = node_curr->parent;
    }

    if (self->verbose_motion)
        fprintf(stdout,"Publishing optimal trajectory (size = %d)\n", n_states);

    // If the optimal trajectory is empty, stop the bot
    if (n_states == 0){
        stop_controller_motion (self);
        return 0;
    }

    erlcm_ref_point_t list[n_states];

    if (optstates_list) {
        GSList *optstates_ptr = optstates_list;
        while (optstates_ptr) {
            int count_path_states = 0;
            node_t *node_curr = (node_t *)(optstates_ptr->data);
            GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
            while (traj_from_parent_ptr) {
                state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
                
                goal.x = state_this->x[0];
                goal.y = state_this->x[1];
                goal.t = state_this->x[2];
                goal.s = self->default_tv;

                count_path_states++;

                if(invalid_root == 0 && !found_commited_root){
                    //check if this is the new root node
                    if(state_this->x[0] == new_root[0] && 
                       state_this->x[1] == new_root[1] && 
                       state_this->x[2] == new_root[2]){
                        found_commited_root = 1;
                        if (self->verbose_motion)
                            fprintf(stderr, "+++++ Found root node Commit point id: %d => Total States : %d\n",  commited_node_id, n_states);
                    }
                    else{
                        commited_node_id++;
                    }
                    
                }
                
                if (count < n_states){//self->num_nodes){
                    list[count] = goal;            
                }
                else{
                    fprintf(stdout, "Hitting Limit\n");
                }

                count++;
                
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
            }

            state_t *state_curr = node_curr->state;
            
            goal.x = state_curr->x[0];
            goal.y = state_curr->x[1];
            goal.t = state_curr->x[2];
            goal.s = self->default_tv;
        
            if (count < n_states){//self->num_nodes){
                list[count] = goal;
            }
            else{
                fprintf(stdout, "Hitting Limit - Outer\n");
            }
            count++;
            
            optstates_ptr = g_slist_next (optstates_ptr);
        }
    }
    if (count == 0)
        return 0;
    
    double xdist, ydist, edist;
    double inc = 0.10;
    for (int i = 1; i < count; i++){
        
        xdist = list[count-1].x - list[i].x;
        ydist = list[count-1].y - list[i].y;
        edist = sqrt ( xdist * xdist + ydist * ydist );
        
        //Ramp (up) speed
        /*if (rampup_speed) {
          if( count < 10 ){
          list[i].s = self->default_tv * inc;
          inc += 0.10;
          }
          }*/
        
        //Ramp (down) speed  
        if( edist < STOP_DIST )
            list[i].s = self->default_tv*(edist+ 0.2/STOP_DIST);
        
    }

    int act_commited_node_id = -1;//count -1;

    if(invalid_root == 0){
        if(found_commited_root){
            act_commited_node_id = commited_node_id; 
        }
        else{
            act_commited_node_id = count -1; 
            fprintf(stderr, " ====== Error - Did Not find Commited Root\n");
        //maybe we should tell here to dump the point 
        }
    }
    else{
        fprintf(stderr, "Invalid Root\n");        
    }
    

    erlcm_ref_point_list_t pub = {
        .num_ref_points = count,
        .commited_point_id = act_commited_node_id,
        .ref_points = list,
        .mode = ERLCM_REF_POINT_LIST_T_NORMAL_MOTION,
        .id = self->goal_id
    };

    if(self->goal_type == 0)
        erlcm_ref_point_list_t_publish (self->lcm, "GOAL_REF_LIST", &pub);
    else if(self->goal_type == 1){
        erlcm_ref_point_list_t_publish (self->lcm, "ELEVATOR_GOAL_REF_LIST", &pub);
    }

    return 1;
}

//publishes optimal path to lcm, this is read by the controller
int 
optmain_publish_optimal_path_to_old_goal (rrtstar_t *self, gboolean rampup_speed,  double *new_root, int invalid_root) {

    int count = 0;
    erlcm_ref_point_t goal;

    int commited_node_id = 0;
    int found_commited_root = 0;
        
    int n_states = 0; 

    GSList *optstates_list = NULL;
    {
        node_t *node_curr = self->opttree->lower_bound_node_to_first_goal;
        while (node_curr) {
            optstates_list = g_slist_prepend (optstates_list, node_curr);
            n_states += g_slist_length (node_curr->traj_from_parent) + 1;
            node_curr = node_curr->parent;
        }
    }

    erlcm_ref_point_t list[n_states];

    if (optstates_list) {
        GSList *optstates_ptr = optstates_list;
        while (optstates_ptr) {
            node_t *node_curr = (node_t *)(optstates_ptr->data);
            GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
            while (traj_from_parent_ptr) {
                state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
                goal.x = state_this->x[0];
                goal.y = state_this->x[1];
                goal.t = state_this->x[2];
                goal.s = self->default_tv;

                if(invalid_root == 0 && !found_commited_root){
                    //check if this is the new root node
                    if(state_this->x[0] == new_root[0] && 
                       state_this->x[1] == new_root[1] && 
                       state_this->x[2] == new_root[2]){
                        found_commited_root = 1;
                        if (self->verbose_motion)
                            fprintf(stderr, "+++++ Found root node Commit point id: %d => Total : %d\n",  commited_node_id, n_states);
                    }
                    else{
                        commited_node_id++;
                    }                    
                }
                
                if (count < n_states){//self->num_nodes){
                    list[count] = goal;            
                }

                count++;
                
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
            }

            state_t *state_curr = node_curr->state;
            
            goal.x = state_curr->x[0];
            goal.y = state_curr->x[1];
            goal.t = state_curr->x[2];
            goal.s = self->default_tv;
        
            if (count < n_states){//self->num_nodes){
                list[count] = goal;
            }

            count++;
            
            optstates_ptr = g_slist_next (optstates_ptr);
        }
    }
    if (count == 0)
        return 0;
    
    double xdist, ydist, edist;
    double inc = 0.10;
    for (int i = 1; i < count; i++){
        
        xdist = list[count-1].x - list[i].x;
        ydist = list[count-1].y - list[i].y;
        edist = sqrt ( xdist * xdist + ydist * ydist );
        
        //Ramp (up) speed
        /*if (rampup_speed) {
          if( count < 10 ){
          list[i].s = self->default_tv * inc;
          inc += 0.10;
          }
          }*/
        
        //Ramp (down) speed  
        if( edist < STOP_DIST )
            list[i].s = self->default_tv*(edist/STOP_DIST);
        
    }

    int act_commited_node_id = -1;//count -1;

    if(invalid_root == 0){
        if(found_commited_root){
            act_commited_node_id = commited_node_id; 
        }
        else{
            act_commited_node_id = count -1; 
        }
    }
    else{
        if (self->verbose_motion)
            fprintf(stderr, "==== (Old Goal :) Error - Did Not find Commited Root\n");
        //maybe we should tell here to dump the point 
    }


    erlcm_ref_point_list_t pub = {
        .num_ref_points = count,
        .commited_point_id = act_commited_node_id,
        .ref_points = list,
        .mode = ERLCM_REF_POINT_LIST_T_NORMAL_MOTION,
        .id = self->goal_id
    };
    erlcm_ref_point_list_t_publish (self->lcm, "GOAL_REF_LIST", &pub);
    
    return 1;
}


int remove_node (rrtstar_t *self, node_t *node_curr) {

    int found_new_lower_bound_node = 0;

    opttree_remove_branch (self->opttree, node_curr);

    // Destroy the kd-tree and rebuild
    int64_t time_start_kd = bot_timestamp_now ();
    kd_clear (self->opttree->kdtree);
    

    self->opttree->lower_bound = DBL_MAX;
    self->opttree->lower_bound_node = NULL;

    //do we trash this ??
    //self->lower_bound_node_to_first_goal = NULL;

    self->opttree->num_nodes = 0;

    
    GSList *nodes_ptr = self->opttree->list_nodes;
    if (!nodes_ptr)
        fprintf (stderr, "ERROR: nodes_ptr is NULL. Function removed entire tree!\n");
    while (nodes_ptr) {
        node_t *node_curr = nodes_ptr->data;
        // We really should be checking these nodes for collision too. Currently, this relies
        // on the lazy_collision_check call to find them. The same is true when updating
        // the lower_bound_node in opttree, which currently does not re-check for collisions
        if ( optsystem_is_reaching_target(self->opttree->optsys, node_curr->state) && (node_curr->distance_from_root < self->opttree->lower_bound)) {
            self->opttree->lower_bound = node_curr->distance_from_root;
            self->opttree->lower_bound_node = node_curr;
            found_new_lower_bound_node = 1;
        }
        kd_insert (self->opttree->kdtree, optsystem_get_state_key (self->opttree->optsys, node_curr->state), node_curr);
        self->opttree->num_nodes++;
        nodes_ptr = g_slist_next (nodes_ptr);
    }
    
    return found_new_lower_bound_node;

}

// Check to see whether the optimal trajectory is in collision
int lazy_collision_check (rrtstar_t *self) {
    opttree_t *opttree = self->opttree;
    
    GSList *opttraj_nodes = NULL;
    
    int num_states = 0;
    node_t *optnode = opttree->lower_bound_node; 
    while (optnode) {
        opttraj_nodes = g_slist_prepend (opttraj_nodes, optnode);
        num_states += g_slist_length (optnode->traj_from_parent) + 1;
        optnode = optnode->parent;
    } 
    

    if (num_states > 0) {

        check_gridmap_update (self->opttree->optsys->grid);

        int first_state = 1;
        double xlast, ylast, tlast;
        double x, y, t;
        double dist = 0;
        GSList *opttraj_nodes_ptr = opttraj_nodes;
        struct check_path_result path_res;
        int is_forward = 1;
        int failsafe = self->opttree->optsys->failsafe_level;
        while (opttraj_nodes_ptr && (dist < self->lazy_collision_check_distance)) {
            node_t *node_curr = (node_t *)(opttraj_nodes_ptr->data);
            GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
            while (traj_from_parent_ptr && (dist < self->lazy_collision_check_distance)) {
                state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
                
                x = state_this->x[0];
                y = state_this->x[1];
                t = state_this->x[2];
                
                if (!first_state) {
                    
                    check_gridmap_check_path (self->opttree->optsys->grid, is_forward, failsafe,
                                              xlast, ylast, tlast,
                                              x, y, t,
                                              &path_res); 
                    
                    // We consider the segment to be in collision if either
                    //   (i)  The line-integrated cost is negative or
                    //   (ii) The maximum obstacle cost along the path exceeds a threshold
                    if ((path_res.cost < 0) || (path_res.obs_max > COMMIT_OBS_MAX_COLLISION) || (path_res.obs_max) < 0) {

                        // Visualize the collision
                        bot_lcmgl_t *lcmgl = self->lcmgl_collision;
                        if (lcmgl) {
                            lcmglColor3f (1.0, 0.0, 0.0);
                            double circle[3] = {x, y, 0.0};
                            lcmglCircle (circle, 0.2);
                            bot_lcmgl_switch_buffer (lcmgl);
                        }

                        // Remove the violating node, and if a new lower_bound_node isn't found (by remove_node),
                        // then set the previous node as the lower_bound_node,
                        // and set the bound to max. This way, there is still a valid path
                        // to use for the committed trajectory and any other node that reaches
                        // the goal should become the new lower_bound_node
                        node_t *parent = node_curr->parent;
                        if (!remove_node (self, node_curr)) {
                            self->opttree->lower_bound = DBL_MAX;
                            self->opttree->lower_bound_node = parent;
                        }
                        g_slist_free (opttraj_nodes);
                        return 1;
                    }
                    
                    dist += sqrt (bot_sq(x-xlast) + bot_sq(y-ylast));
                }
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
                xlast = x;
                ylast = y;
                tlast = t;
                first_state = 0;
            }

            if (dist >= self->lazy_collision_check_distance)
                break;

            x = node_curr->state->x[0];
            y = node_curr->state->x[1];
            t = node_curr->state->x[2];
            if (!first_state) {
                
                check_gridmap_check_path (self->opttree->optsys->grid, is_forward, failsafe,
                                          xlast, ylast, tlast,
                                          x, y, t,
                                          &path_res); 
                
                // We consider the segment to be in collision if either
                //   (i)  The line-integrated cost is negative or
                //   (ii) The maximum obstacle cost along the path exceeds a threshold
                if ((path_res.cost < 0) || (path_res.obs_max > COMMIT_OBS_MAX_COLLISION) || (path_res.obs_max) < 0) {
                    

                    // Visualize the collision
                    bot_lcmgl_t *lcmgl = self->lcmgl_collision;
                    if (lcmgl) {
                        lcmglColor3f (1.0, 0.0, 0.0);
                        double circle[3] = {x, y, 0.0};
                        lcmglCircle (circle, 0.2);
                        bot_lcmgl_switch_buffer (lcmgl);
                    }

                    // Remove the violating node, and if a new lower_bound_node isn't found (by remove_node),
                    // then set the previous node as the lower_bound_node,
                    // and set the bound to max. This way, there is still a valid path
                    // to use for the committed trajectory and any other node that reaches
                    // the goal should become the new lower_bound_node
                    node_t *parent = node_curr->parent;
                    if (!remove_node (self, node_curr)) {
                        self->opttree->lower_bound = DBL_MAX;
                        self->opttree->lower_bound_node = parent;
                    }

                    g_slist_free (opttraj_nodes);
                    return 1;
                }
                dist += sqrt (bot_sq(x-xlast) + bot_sq(y-ylast));
            }

            xlast = x;
            ylast = y;
            tlast = t;
            opttraj_nodes_ptr = g_slist_next (opttraj_nodes_ptr);
        }
        
    }

    g_slist_free (opttraj_nodes);
    
    return 0;


}

// publishes trajectories so they can be rendered

int publish_traj (rrtstar_t *self) {
    
    opttree_t *opttree = self->opttree;

    GSList *opttraj_nodes = NULL;
    
    int num_states = 0;
    node_t *optnode = opttree->lower_bound_node; 
    while (optnode) {
        opttraj_nodes = g_slist_prepend (opttraj_nodes, optnode);
        num_states += g_slist_length (optnode->traj_from_parent) + 1;
        optnode = optnode->parent;
    } 
    

    erlcm_rrt_traj_t *opttraj = (erlcm_rrt_traj_t *) malloc (sizeof (erlcm_rrt_traj_t));

    if (num_states > 0) {
        
        opttraj->num_states = num_states;
        opttraj->states = (erlcm_rrt_state_t *) malloc (opttraj->num_states * sizeof (erlcm_rrt_state_t));
        
        int state_count = 0;
        GSList *opttraj_nodes_ptr = opttraj_nodes;
        while (opttraj_nodes_ptr) {
            node_t *node_curr = (node_t *)(opttraj_nodes_ptr->data);
            GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
            while (traj_from_parent_ptr) {
                state_t *state_this = (state_t *)(traj_from_parent_ptr->data);
                opttraj->states[state_count].x = state_this->x[0];
                opttraj->states[state_count].y = state_this->x[1];
                opttraj->states[state_count].t = state_this->x[2];
                state_count++;
                traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
            }
            opttraj->states[state_count].x = node_curr->state->x[0];
            opttraj->states[state_count].y = node_curr->state->x[1];
            opttraj->states[state_count].t = node_curr->state->x[2];
            state_count ++;
            opttraj_nodes_ptr = g_slist_next (opttraj_nodes_ptr);
        }
    
        self->num_nodes = state_count;
        g_slist_free (opttraj_nodes);
    }
    else {
        opttraj->num_states = 0;
        opttraj->states = NULL;
    }
    
    erlcm_rrt_traj_t_publish (self->lcm, "RRTSTAR_TRAJECTORY", opttraj);
    erlcm_rrt_traj_t_destroy (opttraj);
    
    return 1;
}

void print_committed_traj(rrtstar_t *self){
    if(self->verbose_motion && self->committed_traj){
        GSList *state_ptr = self->committed_traj;
        while (g_slist_next(state_ptr)) {
            state_ptr = g_slist_next (state_ptr);
        }
        state_t *state_end = (state_t *)(state_ptr->data);

        int bot_at_end_of_committed = 0;

        g_mutex_lock (self->at_goal_mutex);
        bot_at_end_of_committed = self->bot_at_end_of_committed;          
        g_mutex_unlock (self->at_goal_mutex);
    
        if(self->verbose_motion){
            fprintf(stdout, "+++++++ End of New Traj Segment  ++++++++ (%.2f,%.2f) At goal : %d\n", 
                    state_end->x[0], state_end->x[1], bot_at_end_of_committed);
        }
    }    
}

// publishes tree so it can be rendered

int publish_tree (rrtstar_t *self) {
        
    opttree_t *opttree = self->opttree;

    GSList *list_nodes = opttree->list_nodes;

    erlcm_rrt_tree_t *tree = (erlcm_rrt_tree_t *) calloc (1, sizeof (erlcm_rrt_tree_t));

    tree->num_nodes = g_slist_length (list_nodes); 
    if (tree->num_nodes > 0) {
        tree->nodes = (erlcm_rrt_node_t *) calloc (tree->num_nodes , sizeof (erlcm_rrt_node_t));
        tree->traj_from_parent = (erlcm_rrt_traj_t *) calloc (tree->num_nodes , sizeof (erlcm_rrt_traj_t));
    }
    else {
        tree->nodes = NULL;
        tree->traj_from_parent = NULL;
    }

    int node_index = 0;
    for (GSList *iter = list_nodes; iter != NULL; iter = g_slist_next(iter)) {
        node_t *node_curr = (node_t *)(iter->data);
        tree->nodes[node_index].nodeid = node_index;
        tree->nodes[node_index].state.x = node_curr->state->x[0];
        tree->nodes[node_index].state.y = node_curr->state->x[1];
        tree->nodes[node_index].state.t = node_curr->state->x[2];
        tree->nodes[node_index].distance_from_root = node_curr->distance_from_root;

        int num_states_in_traj_from_parent = g_slist_length(node_curr->traj_from_parent);
        tree->traj_from_parent[node_index].num_states = num_states_in_traj_from_parent;
        if (num_states_in_traj_from_parent) {
            tree->traj_from_parent[node_index].states 
                = (erlcm_rrt_state_t *) calloc (num_states_in_traj_from_parent , sizeof(erlcm_rrt_state_t));
            int state_index = 0;
            for (GSList *iter2 = node_curr->traj_from_parent; iter2 != NULL; iter2 = g_slist_next (iter2)) {
                state_t *state_curr = (state_t *) (iter2->data); 
                tree->traj_from_parent[node_index].states[state_index].x = state_curr->x[0];
                tree->traj_from_parent[node_index].states[state_index].y = state_curr->x[1];
                tree->traj_from_parent[node_index].states[state_index].t = state_curr->x[2];
                state_index++;
            }

            //fprintf(stderr, "State Ind : %d => %d \n" , tree->traj_from_parent[node_index].num_states, state_index);
        }
        node_index++;
    }
    
    if (tree->num_nodes > 1) {
        tree->num_edges = tree->num_nodes - 1;
        tree->edges = (int32_t **) calloc (tree->num_edges,  sizeof (int32_t *) );
        node_index = 0;
        int edge_index = 0;
        for (GSList *iter = list_nodes; iter != NULL; iter = g_slist_next(iter)) {
            node_t *nodeCurr = (node_t *)(iter->data);
                
            if (nodeCurr->parent) {
                int parent_found = 0;
                int parent_index = 0;
                for (GSList *iterParent = list_nodes; iterParent != NULL; iterParent = g_slist_next(iterParent)) {
                    if ( (node_t *)(iterParent->data) == nodeCurr->parent) {
                        parent_found = 1;
                        break;
                    }
                    parent_index++;
                }
                if (!parent_found & self->verbose_motion)
                    fprintf (stderr, "ERROR: Parent to this node is not in the list\n");                
                tree->edges[edge_index] = (int32_t *) calloc(2 , sizeof(int32_t));
                tree->edges[edge_index][0] = edge_index;
                tree->edges[edge_index][1] = parent_index;
            }
            else{
                //tree->edges[edge_index] = NULL; 
            }
            edge_index++;
        }
    }
    else {
        tree->num_edges = 0;
        tree->edges = NULL;
    }

    erlcm_rrt_tree_t_publish (self->lcm, "RRTSTAR_TREE", tree);

    if(tree){
        erlcm_rrt_tree_t_destroy (tree);
    }

    return 1;
}

void set_operating_region(rrtstar_t *self, int c_ind){
    double pose_range = 5;
        double goal_range = 5;
        double bot_xy[3] = {0,0};
        
        draw_goal(self);

        self->publish_waypoint_status = 1; 

        if(c_ind > 0){
            //we have more than one goal waypoint - and this is not the first one 
            if(self->committed_traj){
                if (self->verbose_motion)
                    fprintf(stdout,"Setting Operating region from the committed Traj\n");
                GSList *state_ptr = self->committed_traj;    

                //we plan from the expected robot position at the end of 
                //the committed traj
                state_ptr = g_slist_last (state_ptr);
                state_t *state_end = (state_t *)(state_ptr->data);
                bot_xy[0] = state_end->x[0];
                bot_xy[1] = state_end->x[1];       
                bot_xy[2] = state_end->x[2];       
            }
            else{
                fprintf(stderr,"Warning : No committed traj (replanning after committed in collision?), planning from current robot pose.\n");
                reset_bot_at_committed_traj (self);
                if (self->bot_pose_last != NULL) {
                    double bot_rpy[3];
                    bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, bot_rpy);

                    bot_xy[0] = self->bot_pose_last->pos[0];
                    bot_xy[1] = self->bot_pose_last->pos[1];
                    bot_xy[2] = bot_rpy[2];
                }
            }
        }
        else{
            if(self->bot_pose_last !=NULL){
                double bot_rpy[3];
                bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, bot_rpy);

                bot_xy[0] = self->bot_pose_last->pos[0];
                bot_xy[1] = self->bot_pose_last->pos[1];

                bot_xy[2] = bot_rpy[2];
            }

            if(self->committed_traj){
                fprintf (stderr, "ERROR: Committed trajectory is not NULL!\n");

                //free commited traj 
                
                GSList *state_ptr = self->committed_traj;
                while (state_ptr){
                    optsystem_free_state (self->opttree->optsys, (state_t *) (state_ptr->data));
                    state_ptr = g_slist_next (state_ptr);
                    
                    g_slist_next(state_ptr);
                }

                g_slist_free(self->committed_traj);
                
                self->committed_traj = NULL;
            }
            reset_bot_at_committed_traj(self);
        }

        if (self->verbose_motion)
            fprintf(stdout,"Setting operating region\n");

        double cg = cos(self->goal_list->goals[c_ind].theta);
        double sg = sin(self->goal_list->goals[c_ind].theta);

        double cr = cos(bot_xy[2]);
        double sr = sin(bot_xy[2]);

        double x_vals[] = {self->goal_list->goals[c_ind].pos[0] + goal_range *cg + goal_range *sg , 
                           self->goal_list->goals[c_ind].pos[0] + goal_range *cg - goal_range *sg , 
                           self->goal_list->goals[c_ind].pos[0] - goal_range *cg + goal_range *sg , 
                           self->goal_list->goals[c_ind].pos[0] - goal_range *cg - goal_range *sg , 
                           bot_xy[0] + pose_range * cr + pose_range * sr, 
                           bot_xy[0] + pose_range * cr - pose_range * sr, 
                           bot_xy[0] - pose_range * cr - pose_range * sr, 
                           bot_xy[0] - pose_range * cr + pose_range * sr};


        double y_vals[] = {self->goal_list->goals[c_ind].pos[1] + goal_range *cg + goal_range *sg,  
                           self->goal_list->goals[c_ind].pos[1] + goal_range *cg - goal_range *sg,  
                           self->goal_list->goals[c_ind].pos[1] - goal_range *cg - goal_range *sg,  
                           self->goal_list->goals[c_ind].pos[1] - goal_range *cg + goal_range *sg,  
                           bot_xy[1] + pose_range * cr + pose_range * sr, 
                           bot_xy[1] + pose_range * cr - pose_range * sr, 
                           bot_xy[1] - pose_range * cr - pose_range * sr, 
                           bot_xy[1] - pose_range * cr + pose_range * sr};
                           

        double max_x = get_max(x_vals, 8);
        double min_x = get_min(x_vals, 8);

        double max_y = get_max(y_vals, 8);
        double min_y = get_min(y_vals, 8);

       
        region_2d_t operating_region = {
            .center = { (max_x + min_x)/2,
                        (max_y + min_y)/2,
            },
            .size = {  
                (max_x - min_x),
                (max_y - min_y),   
            }
        };

        memcpy(&self->operating_region, &operating_region, sizeof(region_2d_t));
        if (self->verbose_motion)
            fprintf(stdout, "Operating Region =>Center : %f,%f\t Size : %f,%f\n", operating_region.center[0], 
                    operating_region.center[1],
                    operating_region.size[0], 
                    operating_region.size[1]);  
        
        optsystem_update_operating_region (self->opttree->optsys, &operating_region);
}

// planning takes place here

void stop_robot(rrtstar_t *self){
    stop_controller_motion(self);
    int stop_iter_start = 0;

    while(is_robot_moving(self)){
        g_mutex_lock (self->stop_iter_mutex);
        stop_iter_start = self->stop_iter; 
        g_mutex_unlock (self->stop_iter_mutex); 
                
        if(stop_iter_start)
            break;
        fprintf(stdout, "Waiting for robot to stop\n"); 
        usleep(50000);    
    }
}

void draw_operating_region(rrtstar_t *self){
    bot_lcmgl_t *lcmgl = self->lcmgl_operating_region;

        //draw region

        if(lcmgl){

            lcmglColor3f(.0, 1.0, 0.0);
            //draw the operating region
            lcmglLineWidth (6);    
            lcmglBegin(GL_LINES);
            lcmglVertex3d(self->operating_region.center[0] + self->operating_region.size[0]/2, 
                          self->operating_region.center[1] + self->operating_region.size[1]/2, 0.05);
            lcmglVertex3d(self->operating_region.center[0] + self->operating_region.size[0]/2, 
                          self->operating_region.center[1] - self->operating_region.size[1]/2, 0.05);
            lcmglEnd();

            lcmglBegin(GL_LINES);
            lcmglVertex3d(self->operating_region.center[0] + self->operating_region.size[0]/2, 
                          self->operating_region.center[1] - self->operating_region.size[1]/2, 0.05);
            lcmglVertex3d(self->operating_region.center[0] - self->operating_region.size[0]/2, 
                          self->operating_region.center[1] - self->operating_region.size[1]/2, 0.05);
            lcmglEnd();
            
            lcmglBegin(GL_LINES);
            lcmglVertex3d(self->operating_region.center[0] - self->operating_region.size[0]/2, 
                          self->operating_region.center[1] - self->operating_region.size[1]/2, 0.05);
            lcmglVertex3d(self->operating_region.center[0] - self->operating_region.size[0]/2, 
                          self->operating_region.center[1] + self->operating_region.size[1]/2, 0.05);

            lcmglEnd();
            
            lcmglBegin(GL_LINES);
            lcmglVertex3d(self->operating_region.center[0] - self->operating_region.size[0]/2, 
                          self->operating_region.center[1] + self->operating_region.size[1]/2, 0.05);

            lcmglVertex3d(self->operating_region.center[0] + self->operating_region.size[0]/2, 
                          self->operating_region.center[1] + self->operating_region.size[1]/2, 0.05);

            lcmglEnd();
            bot_lcmgl_switch_buffer(lcmgl);
        }
}

int handle_first_wp(rrtstar_t *self, int c_ind){
             
    //first goal point and - lets check if the goal is behind us 
    int turn_in_place = 0;
    int no_planning = 0; //set this to skip doing the rrt* 
    int failed_roate = 0; 

    double rpy[3];
    bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, rpy);

    if(self->goal_list->goals[c_ind].do_turn_only){

        self->publish_waypoint_status = 0; 

        turn_in_place =1;
        no_planning = 1;

        //message to controller goes here
        erlcm_ref_point_t list[1];
        erlcm_ref_point_t goal;
                        
        goal.x = self->bot_pose_last->pos[0];//root_state->x[0];
        goal.y = self->bot_pose_last->pos[1];//root_state->x[1];
        goal.t = self->goal_list->goals[c_ind].theta;
        goal.s = 0;
        list[0] = goal;            
                        
        erlcm_ref_point_list_t pub = {
            .num_ref_points = 1,
            .ref_points = list,
            .mode = ERLCM_REF_POINT_LIST_T_TURN_IN_PLACE,
            .id = self->goal_id 
        };
        erlcm_ref_point_list_t_publish (self->lcm, "GOAL_REF_LIST", &pub);
    }
                
    else{
                    
        int goal_rel_check = get_bot_rel_quad ( self->goal_list->goals[c_ind].pos[0], self->goal_list->goals[c_ind].pos[1], self->bot_pose_last->pos[0], self->bot_pose_last->pos[1], rpy[2]);
                    
        if ((goal_rel_check == 2) || (goal_rel_check == 3)){
                        
            fprintf(stdout, "+++++++ Goal is behind the bot! +++++++\n");

            if (self->enable_turn_in_place) {
    
                self->publish_waypoint_status = 0; 
                turn_in_place =1;
                //turn towards the goal 
                double turn_heading = atan2(self->goal_list->goals[c_ind].pos[1] -self->bot_pose_last->pos[1], 
                                            self->goal_list->goals[c_ind].pos[0] -self->bot_pose_last->pos[0]);
                
                //message to controller goes here
                erlcm_ref_point_t list[1];
                erlcm_ref_point_t goal;
                
                goal.x = self->bot_pose_last->pos[0];
                goal.y = self->bot_pose_last->pos[1];
                goal.t = turn_heading;
                goal.s = 0;
                list[0] = goal;            
                
                erlcm_ref_point_list_t pub = {
                    .num_ref_points = 1,
                    .ref_points = list,
                    .mode = ERLCM_REF_POINT_LIST_T_TURN_IN_PLACE,
                    .id = self->goal_id 
                };
                erlcm_ref_point_list_t_publish (self->lcm, "GOAL_REF_LIST", &pub);
            }
        }
    }

    int failed_rotate = 0;
                
    if(turn_in_place){              
        //this doesnt wait till there is a response 
        fprintf(stdout, "Waiting for robot to finish motion\n"); 
        //for now wait till the bot is stopped 
        int stop_iter = 0;                   

        while(!stop_iter){
            g_mutex_lock (self->stop_iter_mutex);
            stop_iter = self->stop_iter; 
            g_mutex_unlock (self->stop_iter_mutex); 

            usleep(50000);
            if(self->goal_status){
                if((self->goal_id == self->goal_status->id) && 
                   (self->goal_status->status == ERLCM_RRT_GOAL_STATUS_T_REACHED ||
                    self->goal_status->status == ERLCM_RRT_GOAL_STATUS_T_FAILED)){

                    if(self->goal_status->status == ERLCM_RRT_GOAL_STATUS_T_FAILED){
                        failed_rotate = 1;
                    }
                    
                    while(is_robot_moving(self)){
                        g_mutex_lock (self->stop_iter_mutex);
                        stop_iter = self->stop_iter; 
                        g_mutex_unlock (self->stop_iter_mutex); 
                        fprintf(stdout, "Waiting for robot to stop\n");
                        usleep(50000);    
                        if(stop_iter){
                            break;
                        }
                    }
                    fprintf(stderr, "Robot Stopped - Starting to plan actual Path\n");
                    //maybe send message out here 
                    //so we can ask the person to stand behind 
                    break; 
                }
            }
        } 
    }

    if(no_planning){
        fprintf(stderr, "Done turning - stop the loop and go back to the start. Robot has arrived at the current goal\n");
                    
        erlcm_speech_cmd_t msg;
        msg.utime = bot_timestamp_now();
        msg.cmd_type = "WAYPOINT_STATUS"; 
        if(failed_rotate){
            msg.cmd_property = "FAILED"; 
        }
        else{
            msg.cmd_property = "REACHED"; 
        }
        erlcm_speech_cmd_t_publish (self->lcm, "WAYPOINT_STATUS", &msg);
                    
        g_mutex_lock(self->running_mutex);
        self->is_running = 0;
        g_mutex_unlock(self->running_mutex);

        //g_mutex_lock (self->plan_mutex);
        return 1;
    }
    return 0;
}

void set_rootstate(rrtstar_t *self, int c_ind, int *committed_in_collision){
    // Set the root state 
    state_t *root_state = optsystem_new_state (self->opttree->optsys);
    
    if(c_ind > 0){
        //we have more than one goal waypoint - and this is not the first one 
        if(self->committed_traj){
            if (self->verbose_motion)
                fprintf(stdout, "Setting Root Node from Committed traj\n");
            GSList *state_ptr = self->committed_traj;
            state_ptr = g_slist_last (state_ptr);

            state_t *state_end = (state_t *)(state_ptr->data);
            root_state->x[0] = state_end->x[0];
            root_state->x[1] = state_end->x[1];       
            root_state->x[2] = state_end->x[2];     
        }      
        else{
            if (self->verbose_motion)
                fprintf(stderr,"Error : No committed traj\n");
            if(self->bot_pose_last !=NULL){
                root_state->x[0] = self->bot_pose_last->pos[0];
                root_state->x[1] = self->bot_pose_last->pos[1];
                double bot_rpy[3];
                bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, bot_rpy);
                root_state->x[2] = bot_rpy[2];
            }
            else{
                if (self->verbose_motion)
                    fprintf(stdout,"No pose message: Using the initial pose from the message as the root\n");
                root_state->x[0] = 0;
                root_state->x[1] = 0;
                root_state->x[2] = 0; 
            }
        }
    }
    else if(self->bot_pose_last !=NULL){
        root_state->x[0] = self->bot_pose_last->pos[0];
        root_state->x[1] = self->bot_pose_last->pos[1];
        double bot_rpy[3];
        bot_quat_to_roll_pitch_yaw (self->bot_pose_last->orientation, bot_rpy);
        root_state->x[2] = bot_rpy[2];
    } 
    else{
        root_state->x[0] = 0;
        root_state->x[1] = 0;
        root_state->x[2] = 0; 
    }

    if (self->verbose_motion)
        fprintf(stdout," Root Node (%f,%f,%f)\n",root_state->x[0], root_state->x[1], root_state->x[2] );

        
    //set the root node if this is the first goal point - or if the tree was trashed after wp
    if(c_ind==0 || self->trash_tree_on_wp || *committed_in_collision){
        if (self->verbose_motion)
            fprintf(stderr, "Resetting Root State\n");
            
        opttree_set_root_state (self->opttree, root_state);
        optsystem_free_state (self->opttree->optsys, root_state);

        *committed_in_collision = 0;
    }
}

void update_goal_region(rrtstar_t *self, int c_ind){
    double theta_toll = 2* M_PI;
    
    if(self->goal_list->goals[c_ind].use_theta){
        theta_toll = self->goal_list->goals[c_ind].heading_tol * 2; 
    }
    
    // Report the goal region
    region_3d_t goal_region = {
        .center = {
            self->goal_list->goals[c_ind].pos[0],
            self->goal_list->goals[c_ind].pos[1],
            self->goal_list->goals[c_ind].theta,
        },
        .size = {  
            self->goal_list->goals[c_ind].size[0],
            self->goal_list->goals[c_ind].size[1],    
            theta_toll,
        }
    };

    if(self->verbose_motion){
        printf("goal from message %f\n",self->goal_list->goals[c_ind].pos[0]);
        printf("goal from message %f\n",self->goal_list->goals[c_ind].pos[1]);
        printf("goal from message %f\n", self->goal_list->goals[c_ind].size[0]);
        printf("goal from message %f\n", self->goal_list->goals[c_ind].size[1]);
    }
    
    //update the goal 
    optsystem_update_goal_region (self->opttree->optsys, &goal_region);

    // set goal type
    if (self->goal_id == 1)
        optsystem_update_goal_type (self->opttree->optsys, TRUE);
    else
        optsystem_update_goal_type (self->opttree->optsys, FALSE);
}

int stop_robot_in_collision_traj(rrtstar_t *self){
    while (is_robot_moving (self)) {
        int stop_iter = 0;
        g_mutex_lock (self->stop_iter_mutex);
        stop_iter = self->stop_iter; 
        g_mutex_unlock (self->stop_iter_mutex); 
        if(stop_iter){
            break;
        }
        estop_controller (self);
        fprintf (stdout, "Telling the bot to stop\n");
        usleep(50000);                
    }

    reset_bot_at_committed_traj(self);

    // Free the committed trajectory
    GSList *state_ptr = self->committed_traj;
    while (state_ptr) {
        state_t *state_curr = (state_t *)(state_ptr->data);
        optsystem_free_state (self->opttree->optsys, state_curr);
        state_ptr = g_slist_next (state_ptr);
    }
    g_slist_free (self->committed_traj);
    self->committed_traj = NULL;
    return 1;
}

gpointer* 
on_planning_thread (gpointer data) {
    rrtstar_t *self = (rrtstar_t *) data;

    printf ("Start planning thread\n");

    stop_controller_motion(self);

    int committed_in_collision = 0;

    int failed_last_attempt = 0;
    int second_failure = 0;

    while (1) { 
        // Wait for the command message
        g_mutex_lock (self->plan_mutex);
        
        //turn on the check gridmap 
        check_gridmap_on(self->opttree->optsys->grid);

        g_mutex_lock(self->running_mutex);
        self->is_running = 1;
        g_mutex_unlock(self->running_mutex);
       
        int c_ind = self->current_goal_ind; 

        if(c_ind==0 && self->is_new_goal){
            failed_last_attempt = 0;
            second_failure = 0;
            self->goal_id +=1;
            self->is_new_goal = 0;

            if(self->goal_id == INT32_MAX)
                self->goal_id = 0;
        }

        //the planner failed to find path in last go - reducing footprint 
        if(failed_last_attempt){
            fprintf(stdout, "Having trouble finding path, increasing failsafe\n");
            //taking failsafe up - reduces the footprint
            self->opttree->optsys->failsafe_level = 2; 
        }
        else{
            self->opttree->optsys->failsafe_level = self->basic_failsafe;
        }
        
        //new goal - stop the robot 
        if(c_ind == 0){ 
            stop_robot(self);
        }

        //we need to use the same method to do turn-in-place for inside elevator 
        if(self->bot_pose_last !=NULL){   
            if(c_ind ==0){
                //handle the first waypoint - either turn towards the goal if looking the other way - or turn in place 
                if(handle_first_wp(self, c_ind)){
                    continue;
                }
            }
        }  

        if (self->verbose_motion)
            fprintf(stdout," At start - planning to goal\n");

        // Buffers to place around goal and bot pose. The rectangle that contains both
        // defines the operating region within which the RRT* samples. Ideally, the calling
        // process should be able to set these based upon a failsafe (e.g., if we fail 
        // to find a solution, try a larger region).

        //update the operating region
        //operating region should include the goal and the bot and space inbetween 
        set_operating_region(self, c_ind);
        
        //draw the operating region 
        draw_operating_region(self);

        //update the goal-region using the current waypoint
        update_goal_region(self, c_ind);
        
        //set the root state - could be the current robot position 
        //or the last point in the commited traj
        set_rootstate(self, c_ind, &committed_in_collision);
        
        if(c_ind==0){
            // Initialize the rrtstar library
            self->opttree->run_rrtstar = TRUE;
            self->opttree->ball_radius_constant = 50.0;
            self->opttree->ball_radius_max = 8.0;
            self->opttree->target_sample_prob_before_first_solution = 0.2;//was 0.2
            self->opttree->target_sample_prob_after_first_solution = 0.05;
    
            // Initialize the random number generator
            int seed = time (NULL);
            //printf ("Random number generator initialized with seed %d\n", seed);
            srand ( seed );
        }

        // RUN THE MAIN LOOP    
        self->iteration_no = 0;
        int num_iterations = 0;
        int iteration_limit = self->config.iteration_limit;
        int64_t time_limit = self->config.time_limit;
    
        if ( (iteration_limit < 0) && (time_limit < 0) ) {
            g_mutex_lock (self->plan_mutex);
            continue;
        }

        int tree_pub_limit = self->config.tree_pub_iteration_limit;
        int traj_pub_limit = self->config.traj_pub_iteration_limit;

        int tree_pub_iteration_last = 0;
        int traj_pub_iteration_last = 0;

        self->time_start = bot_timestamp_now ();
        double time_start = self->time_start;

        double time_final = 3.0;

        double time_final_fail = 20.0;
    
        fprintf(stdout,"Looking for Path\n");
        int stop_iter = 0;

        //look for a feasible path
        while (((num_iterations < iteration_limit) || (! (self->opttree->lower_bound_node))) 
               && !stop_iter) {

            if ((self->iteration_no)%50 == 0) {
                g_mutex_lock (self->stop_iter_mutex);
                stop_iter = self->stop_iter; 
                g_mutex_unlock (self->stop_iter_mutex); 
            }

            // Carry out a single iteration with the given parameters
            opttree_iteration (self->opttree);
            num_iterations++;
            self->iteration_no = num_iterations; 

            if ((self->iteration_no)%50000 == 0) 
                fprintf(stdout,"At iteration number %d\n", self->iteration_no);

            // Every 100 iterations, if we have a solution call branch-and-bound on the tree
            if (self->perform_branch_and_bound && ((self->iteration_no)%100 == 0) && (self->opttree->lower_bound_node != NULL)) {
                publish_tree (self);
                opttree_branch_and_bound (self->opttree);
                publish_tree (self);
            } 

            if ( is_bot_at_end_of_committed_traj (self) == 1 ) {

                //use the path to the old solution 
                double new_root[3] = {.0,.0,.0}; 
                int no_new_root = opttree_get_commit_end_point_to_old_goal(self->opttree, self->config.commit_time , new_root);

                if (self->verbose_motion)
                    fprintf(stderr," ---------- Using earlier solution. New Root is : %f,%f,%f : %d\n",  
                            new_root[0], new_root[1], new_root[2], no_new_root);  

                optmain_publish_optimal_path_to_old_goal(self, FALSE, new_root, no_new_root);
          
                int all_committed = 0;
                // Free the committed traj
                GSList *state_ptr = self->committed_traj;
                while (state_ptr) {
                    state_t *state_curr = (state_t *)(state_ptr->data);
                    optsystem_free_state (self->opttree->optsys, state_curr);
                    state_ptr = g_slist_next (state_ptr);
                }
                g_slist_free (self->committed_traj);
                // Update the committed traj
        
                reset_bot_at_committed_traj (self);
        
                //publish the path to the old path node - this will screw up with branch and bound
                self->committed_traj = opttree_commit_traj_to_old_goal (self->opttree, self->config.commit_time , &all_committed);                
                publish_tree (self);
                tree_pub_iteration_last = num_iterations;
            
                print_committed_traj(self);
            }

        
            // Publish the tree message if the interval is correct
            int64_t time_now = bot_timestamp_now ();
            double time_diff = ((double)(time_now - time_start))/1000000.0;

            if (time_diff > time_final && self->opttree->lower_bound_node) { 
                //break using the time limit - if we have a solution
                if (self->verbose_motion)
                    fprintf (stdout, "time_diff = %.2f sec > %.2f = time_final and we have a solution. Breaking out of first while() loop.\n",
                             time_diff, time_final);
                break;
            }
        
            if(time_diff > time_final_fail) {
                fprintf(stderr, "Failed to find an initial solution in time_diff = %.2f > %.2f = time_final_fail. Breaking out of first while() loop.\n",
                        time_diff, time_final_fail); 
                break; 
            }
            
            if (self->verbose_screen) {
                printf ("%7d:: Time : %5.5lf, Lower bound: %5.5lf, num_nodes: %d\n", 
                        num_iterations,
                        time_diff, 
                        self->opttree->lower_bound > 100000000.0 ? 100000000.0:self->opttree->lower_bound, 
                        self->opttree->num_nodes);
            }

            // publish tree and trajectories every now and then (for rendering)
            if (tree_pub_limit > 0)  
                if (num_iterations - tree_pub_iteration_last >= tree_pub_limit) {
                    publish_tree (self);
                    tree_pub_iteration_last = num_iterations;
                }
            if (traj_pub_limit > 0)
                if (num_iterations - traj_pub_iteration_last >= traj_pub_limit) {
        
                    publish_traj (self);
                    traj_pub_iteration_last = num_iterations;
                }        
        }

        opttree_reset_old_goal_node(self->opttree);

        //now we need to wait until the robot has reached the end of the committed trajctory
        if(c_ind > 0 && self->committed_traj && !stop_iter){
            while(!(is_bot_at_end_of_committed_traj (self)==1) && !stop_iter){
                g_mutex_lock (self->stop_iter_mutex);
                stop_iter = self->stop_iter; 
                g_mutex_unlock (self->stop_iter_mutex); 

                //add a break when the robot has been stopped from outside 
                /*if(self->goal_status){
                  if((self->goal_id == self->goal_status->id) && 
                  self->goal_status->status == ERLCM_RRT_GOAL_STATUS_T_STOPPED)){
                  break;
                  }*/
                //ideally we should keep improving
                usleep(50000);
            }
            //free the committed traj

            GSList *state_ptr = self->committed_traj;
            while (state_ptr) {
                state_t *state_curr = (state_t *)(state_ptr->data);
                optsystem_free_state (self->opttree->optsys, state_curr);
                state_ptr = g_slist_next (state_ptr);
            }
            g_slist_free (self->committed_traj);

            self->committed_traj = NULL;
            reset_bot_at_committed_traj (self);        
        }

        publish_traj (self);

        double new_root[3] = {.0,.0,.0}; 
        int no_new_root = opttree_get_commit_end_point(self->opttree, self->config.commit_time , new_root);

        if (self->verbose_motion)
            fprintf(stderr," ---------- Bot reached end of committed trajectory. New Root is : %f,%f,%f : %d\n",  
                    new_root[0], new_root[1], new_root[2], no_new_root);  

        optmain_publish_optimal_path (self, TRUE, new_root, no_new_root);
    
        int all_committed_initial = 0;

        // Free the committed traj
        if(self->committed_traj){
            GSList *state_ptr = self->committed_traj;
            while (state_ptr) {
                state_t *state_curr = (state_t *)(state_ptr->data);
                optsystem_free_state (self->opttree->optsys, state_curr);
                state_ptr = g_slist_next (state_ptr);
            }
            g_slist_free (self->committed_traj);
        }

        if(self->opttree->lower_bound_node == NULL){
            
            if(failed_last_attempt) {
                if (self->verbose_motion)
                    fprintf(stderr,"No valid solution found - second failure\n");
                second_failure = 1;
                failed_last_attempt = 0; 
            }
            else{
                if (self->verbose_motion)
                    fprintf(stderr,"No valid solution found - first faliure\n");
                failed_last_attempt = 1;
            }
        }
        else{
            //clear the failed attempt if we have a solution 
            failed_last_attempt = 0;
        }

       self->committed_traj = opttree_commit_traj (self->opttree,  self->config.commit_time , &all_committed_initial);

        if(all_committed_initial){
            if(self->current_goal_ind == self->goal_list->num_goals-1){
                self->commited_to_final_goal = TRUE;
            }
        }

        // Check to see whether the committed trajectory is in collision
        committed_in_collision = is_committed_trajectory_in_collision (self);

        // Check to see whether the optimal trajectory is in collision
        //int opt_in_collision = opt_traj_in_collision (self);
        //if (opt_in_collision)
        //    fprintf (stdout, "Optimal trajectory is in collision\n");

        if (committed_in_collision) {
            if(stop_robot_in_collision_traj(self)){
                break;
            }
        }        
    
        publish_tree (self);
        tree_pub_iteration_last = num_iterations;
    
        print_committed_traj(self);
    
        double time_last_commit = time_start;

        if (!self->committed_traj) {
            fprintf (stdout, "Failed to find an initial solution\n");
            stop_iter = 1;
        }
    
        //improve the path 
        while ((num_iterations < iteration_limit && !all_committed_initial) 
               && !stop_iter) {

            if ((self->iteration_no)%50 == 0) {
                g_mutex_lock (self->stop_iter_mutex);
                stop_iter = self->stop_iter; 
                g_mutex_unlock (self->stop_iter_mutex); 
            }

            if (!self->committed_traj){
                fprintf(stdout, "No Committed Traj\n");
                int in_collision;
                //set_rootstate (self, c_ind, &in_collision);

                self->committed_traj = opttree_commit_traj (self->opttree,  self->config.commit_time , &all_committed_initial);

                // If there is still no committed trajectory, it is likely that
                // we've reached the end of the previous trajectory and don't have
                // a path to the goal
                if (!self->committed_traj) {
                    fprintf (stderr, "+++++++ Trashing the tree +++++++\n");
                    failed_last_attempt = 1;
                    g_mutex_unlock (self->plan_mutex);
                    break;
                }

                if(all_committed_initial) {
                    if(self->current_goal_ind == self->goal_list->num_goals-1){
                        self->commited_to_final_goal = TRUE;
                    }
                }

                if (self->committed_traj) {
                    double new_root[3] = {.0,.0,.0}; 
                    int no_new_root = opttree_get_commit_end_point(self->opttree, self->config.commit_time , new_root);
                    
                    if (self->verbose_motion)
                        fprintf(stderr," ---------- Setting new path with new root : %f,%f,%f : %d\n",
                                new_root[0], new_root[1], new_root[2], no_new_root);

                    optmain_publish_optimal_path (self, FALSE, new_root, no_new_root);

                }

                //break;
            }

            // Carry out a single iteration with the given parameters
            opttree_iteration (self->opttree);
            num_iterations++;
            self->iteration_no = num_iterations; 

            if (self->perform_branch_and_bound && (self->iteration_no)%200 == 0) {
                opttree_branch_and_bound (self->opttree);
            } 

            if (self->perform_lazy_collision_check && (self->iteration_no)%100 == 0) {
                if (lazy_collision_check (self)) {
                    if (self->verbose_motion)
                        fprintf (stdout, "Optimal trajectory is in collision\n");

                    // Publish the path to the new lower_bound_node (note that
                    // this path may not reach the goal, but if that is the case
                    // the cost is max and the path should be replaced as soon
                    // as a new path to the goal is found
                    double new_root[3] = {.0,.0,.0}; 
                    /* int no_new_root = opttree_get_commit_end_point(self->opttree, self->config.commit_time , new_root); */
                    
                    /* if (self->verbose_motion) */
                    /*     fprintf(stderr," ---------- Setting new path with new root : %f,%f,%f : %d\n",   */
                    /*             new_root[0], new_root[1], new_root[2], no_new_root);   */

                    /* optmain_publish_optimal_path (self, FALSE, new_root, no_new_root); */
                }
            }

            // Publish the tree message if the interval is correct
            int64_t time_now = bot_timestamp_now ();
            double time_diff = ((double)(time_now - time_last_commit))/1000000.0;        
            if ( is_bot_at_end_of_committed_traj (self) == 1 ) {    
                if(is_bot_near_goal (self, &(self->goal_list->goals[self->current_goal_ind])) == 1 
                   && (self->current_goal_ind < self->goal_list->num_goals-1) ) {
                    //if we are near the goal and there are more goals in the list
                    //committing part of the traj and setting new goal
                    
                    double new_root[3] = {.0,.0,.0}; 
                    int no_new_root = opttree_get_commit_end_point(self->opttree, self->config.commit_time , new_root);
                    
                    if (self->verbose_motion)
                        fprintf(stderr," ---------- Bot reached end of committed trajectory and is near goal. New Root is : %f,%f,%f : %d\n",  
                                new_root[0], new_root[1], new_root[2], no_new_root);  

                    optmain_publish_optimal_path (self, FALSE, new_root, no_new_root);

                    // Free the committed traj
                    GSList *state_ptr = self->committed_traj;
                    while (state_ptr) {
                        state_t *state_curr = (state_t *)(state_ptr->data);
                        optsystem_free_state (self->opttree->optsys, state_curr);
                        state_ptr = g_slist_next (state_ptr);
                    }
                    g_slist_free (self->committed_traj);
                    // Update the committed traj
                    int all_committed = 0;
            
                    if(!self->trash_tree_on_wp){                  
                        self->committed_traj = opttree_commit_traj (self->opttree, self->config.commit_time, &all_committed);
                    }
                    else{
                        self->committed_traj = opttree_commit_traj_all (self->opttree);
                    }

                    // When the lazy collision check detects that the
                    // optimal path is in collision, it may initially
                    // set the lower_bound_node to an intermediate
                    // node along the path and set the lower bound
                    // cost to DBL_MAX. If this node is within the
                    // commit time, opttree_commit_traj will set
                    // all_committed to 1. 
                    if(all_committed && self->opttree->lower_bound < DBL_MAX){
                        if(self->current_goal_ind == self->goal_list->num_goals-1){
                            self->commited_to_final_goal = TRUE;
                        }
                    }

                    // Check to see whether the committed trajectory is in collision
                    committed_in_collision = is_committed_trajectory_in_collision (self);
                    if (committed_in_collision) {
                        fprintf (stdout, "Committed trajectory is in collision. Telling the bot to stop\n");
                        while (is_robot_moving (self)) {
                            estop_controller (self);
                            usleep(50000);
                        }

                        reset_bot_at_committed_traj(self);        
                        
                        // Free the committed trajectory
                        GSList *state_ptr = self->committed_traj;
                        while (state_ptr) {
                            state_t *state_curr = (state_t *)(state_ptr->data);
                            optsystem_free_state (self->opttree->optsys, state_curr);
                            state_ptr = g_slist_next (state_ptr);
                        }
                        g_slist_free (self->committed_traj);
                        self->committed_traj = NULL;
                        break;
                    }


                    publish_tree (self);
                    tree_pub_iteration_last = num_iterations;

                    reset_bot_at_committed_traj(self);        

                    print_committed_traj(self);

                    time_last_commit = time_now;            
                    break;
                }
                else{//commit a bit more of the traj  

                    double new_root[3] = {.0,.0,.0}; 
                    int no_new_root = opttree_get_commit_end_point(self->opttree, self->config.commit_time , new_root);

                    if (self->verbose_motion)
                        fprintf(stderr," ---------- Bot reached end of committed trajectory but not near goal. New Root is : %f,%f,%f : %d\n",  
                                new_root[0], new_root[1], new_root[2], no_new_root);  

                    optmain_publish_optimal_path (self, FALSE, new_root, no_new_root);
          
                    int all_committed = 0;
                    // Free the committed traj
                    GSList *state_ptr = self->committed_traj;
                    while (state_ptr) {
                        state_t *state_curr = (state_t *)(state_ptr->data);
                        optsystem_free_state (self->opttree->optsys, state_curr);
                        state_ptr = g_slist_next (state_ptr);
                    }
                    g_slist_free (self->committed_traj);
                    // Update the committed traj
                    self->committed_traj = opttree_commit_traj (self->opttree, self->config.commit_time, &all_committed);

                    // When the lazy collision check detects that the
                    // optimal path is in collision, it may initially
                    // set the lower_bound_node to an intermediate
                    // node along the path and set the lower bound
                    // cost to DBL_MAX. If this node is within the
                    // commit time, opttree_commit_traj will set
                    // all_committed to 1. 
                    if(all_committed && self->opttree->lower_bound < DBL_MAX){
                        if(self->current_goal_ind == self->goal_list->num_goals-1){
                            self->commited_to_final_goal = TRUE;
                        }
                    }

                    // Check to see whether the committed trajectory is in collision
                    committed_in_collision = is_committed_trajectory_in_collision (self);
                    if (committed_in_collision) {
                        fprintf (stdout, "Committed trajectory is in collision. Telling the bot to stop\n");
                        while (is_robot_moving (self)) {
                            estop_controller (self);
                            usleep(50000);
                        }


                        reset_bot_at_committed_traj(self);        
                        // Free the committed trajectory
                        GSList *state_ptr = self->committed_traj;
                        while (state_ptr) {
                            state_t *state_curr = (state_t *)(state_ptr->data);
                            optsystem_free_state (self->opttree->optsys, state_curr);
                            state_ptr = g_slist_next (state_ptr);
                        }
                        g_slist_free (self->committed_traj);
                        self->committed_traj = NULL;
                        break;
                    }

        
                    reset_bot_at_committed_traj (self);

                    publish_tree (self);
                    tree_pub_iteration_last = num_iterations;

                    print_committed_traj(self);

                    time_last_commit = time_now;

                    if(all_committed && (self->opttree->lower_bound < DBL_MAX)){
                        break;
                    }
                }
            }
        
            if (self->verbose_screen) {
                printf ("%7d:: Time : %5.5lf, Lower bound: %5.5lf, num_nodes: %d\n", 
                        num_iterations,
                        time_diff, 
                        self->opttree->lower_bound > 100000000.0 ? 100000000.0:self->opttree->lower_bound, 
                        self->opttree->num_nodes);
            }

            if (tree_pub_limit > 0)  
                if (num_iterations - tree_pub_iteration_last >= tree_pub_limit) {
                    publish_tree (self);
                    tree_pub_iteration_last = num_iterations;
                }
            if (traj_pub_limit > 0)
                if (num_iterations - traj_pub_iteration_last >= traj_pub_limit) {
                    publish_traj (self);
                    traj_pub_iteration_last = num_iterations;
                }
        }

        if (self->config.tree_pub_final)
            publish_tree (self);
        if (self->config.traj_pub_final)
            publish_traj (self);
    
        update_local_goal_list(self);
        erlcm_goal_t *final_goal = &(self->goal_list->goals[self->goal_list->num_goals-1]);        
        int bot_at_goal = is_bot_at_goal(self,final_goal);
    
        if (self->verbose_motion)
            fprintf(stdout, "Current Goal Ind : %d Num Goals : %d Bot at Goal : %d\n", self->current_goal_ind , self->goal_list->num_goals-1, bot_at_goal);

        //do not reinitialize the tree if we are not at the last goal point (and not set to trash tree at every wp or were broken out of the planning loop)    
    
        // Are we here because the committed trajectory was in collision? If so, reinitialize the tree
        if (((committed_in_collision) && !bot_at_goal && !stop_iter) || failed_last_attempt) {
            fprintf(stderr, " ++++++ Trashing the tree ++++++\n");
            opttree_reinitialize (self->opttree);
            g_mutex_unlock (self->plan_mutex);
        }
        else if((self->current_goal_ind < (self->goal_list->num_goals-1) && !self->trash_tree_on_wp) && !bot_at_goal && !stop_iter){    
            if(second_failure){
                fprintf(stderr,"Two failures - going to next waypoint : %d - %d\n", self->current_goal_ind, self->goal_list->num_goals);
            }
            //we have more goals - move on to the next one 
            self->current_goal_ind++; 
            update_local_goal_list(self);

            opttree_reset (self->opttree);
            g_mutex_unlock (self->plan_mutex);           
        }
        else{

            if(second_failure && (self->current_goal_ind == (self->goal_list->num_goals-1))){
                fprintf(stderr,"Second faliure on the last goal - send out failed to get to goal msg - stopping\n");
                 erlcm_speech_cmd_t msg;
                 msg.utime = bot_timestamp_now();
                 msg.cmd_type = "WAYPOINT_STATUS"; 
                 msg.cmd_property = "FAILED"; 
                 erlcm_speech_cmd_t_publish (self->lcm, "WAYPOINT_STATUS", &msg);
                
            }
               
            int stop_iter = 0;

            while(!stop_iter){//is_robot_moving(self)){                        
                usleep(50000);

                g_mutex_lock (self->stop_iter_mutex);
                stop_iter = self->stop_iter; 
                g_mutex_unlock (self->stop_iter_mutex); 

                if(self->goal_status){
                    //break if stop is called                    
                    if((self->goal_id == self->goal_status->id) && 
                       (self->goal_status->status == ERLCM_RRT_GOAL_STATUS_T_REACHED)){
                        
                        while(is_robot_moving(self)){
                            g_mutex_lock (self->stop_iter_mutex);
                            stop_iter = self->stop_iter; 
                            g_mutex_unlock (self->stop_iter_mutex); 
                            
                            if(stop_iter){
                                break;
                            }
                            fprintf(stdout, "Waiting for robot to stop\n");
                            usleep(50000);    
                        }
                        
                        if(self->publish_waypoint_status){
                            fprintf(stdout, "Robot has arrived at the current goal\n"); 

                            erlcm_speech_cmd_t msg;
                            msg.utime = bot_timestamp_now();
                            msg.cmd_type = "WAYPOINT_STATUS"; 
                            msg.cmd_property = "REACHED"; 
                            erlcm_speech_cmd_t_publish (self->lcm, "WAYPOINT_STATUS", &msg);
                        }
                        
                        fprintf(stderr, "Robot Stopped - Publishing waypoint status\n");
                        g_mutex_lock(self->running_mutex);
                        self->is_running = 0;
                        g_mutex_unlock(self->running_mutex);

                        break; 
                    }
                }
            }
            
            //we should send a bot at goal message - either here or from the controller 
            
            opttree_reinitialize (self->opttree);
            printf ("============ RRTSTAR is done =============\n");
            //pausing the map
            if(!self->continuous_map_updates){
                check_gridmap_off(self->opttree->optsys->grid);
            }
        }

        g_mutex_lock (self->stop_iter_mutex);
        self->stop_iter = 0; 
        g_mutex_unlock (self->stop_iter_mutex);
    }    
    
    printf ("End planning thread\n");
    
    return NULL;
}


static void usage(int argc, char ** argv)
{
    fprintf (stderr, "Usage: %s [OPTIONS]\n"
             "RRT* motion planner\n"
             "\n"
             "Options:\n"
             "  -s, --speed <SPEED>                Nominal speed (not functional?) \n"
             "  -w, --width <NUM>                  Additional width to add for check_gridmap (default: %.2f)\n"
             "  -l, --local                        Populate the region of the gridmap around the robot\n"
             "                                     Based only on detected obstacles, ignoring SLAM occupancy map\n"
             "  -n, --no-map                       Don't use map for collision checking\n"
             "  -c, --continuous                   Update the gridmap even when not planning (?)\n"
             "  -b, --branch-and-bound             Use branch and bound. This may cause bot to stop\n"
             "                                     when lazy collision check is enabled\n"
             "                                     and the optimal path is in collision\n"
             "  -D, --dont-clear                   Don't clear occupied cells in map\n"
             "                                     when there are no corresponding laser returns\n"
             "  -l, --large-failsafe               Use a large failsafe, which relaxes constraints\n"
             "  -t, --trash-tree                   Trash the search tree after reaching a waypoint\n"
             "  -d, --draw                         Draw path queries\n"
             "  -N, --no-turn-in-place             Disable turn in place for goals behind robot\n"
             "  -L, --perform-lazy-collision-check Perform lazy collision check of optimal trajectory\n"
             "  -h, --help                         Print this help and exit\n"
             "  -v, --verbose                      Verbose output\n", argv[0], DEFAULT_CHECK_GRIDMAP_WIDTH_BUFFER);
}


int main (int argc, char **argv) {
   
    setlinebuf(stdout);

    /*
     * The following options are used for check_gridmap_create
     *
     * sensing_only_local:        Specify whether to populate the area of the gridmap around the
     *                            robot based only on detected obstacles and not use the 
     *                            SLAM occupancy map.
     *
     * no_map/sensing_only_small: Generate spatially compact gridmap that only renders
     *                            locally-perceived obstacles. (Added for door open/closed detection.)
     *
     * clear_using_laser :        Clear the map using the extent of the laser
     *
     */
    gboolean sensing_only_local = FALSE;
    gboolean draw = FALSE;
    gboolean reset_nom_speed = FALSE;
    gboolean verbose = FALSE;
    gboolean trash_tree_on_wp = FALSE;
    gboolean continuous_map_updates = FALSE; 
    gboolean large_failsafe = FALSE; 
    gboolean clear_using_laser = TRUE; 
    gboolean enable_turn_in_place = TRUE;
    gboolean perform_lazy_collision_check = FALSE;
    gboolean perform_branch_and_bound = FALSE;
    gboolean no_map = FALSE; 
    double nom_speed = 0;
    double check_gridmap_width_buffer = DEFAULT_CHECK_GRIDMAP_WIDTH_BUFFER;
    char *optstring = "s:w:lvtcbdfDnNLh";
    char c;
    struct option long_opts[] = { 
        { "speed", required_argument, 0, 's' },
        { "width", required_argument, 0, 'w' },
        { "local", no_argument, 0, 'l' },
        { "no-map", no_argument, 0, 'n' },
        { "verbose", no_argument, 0, 'v' },
        { "draw", no_argument, 0, 'd' },
        { "dont-clear", no_argument, 0, 'D' },
        { "branch-and-bound", no_argument, 0, 'b' },
        { "large-failsafe", no_argument, 0, 'f' },
        { "continuous", no_argument, 0, 'c' },
        //trash tree is broken now - fix
        { "trash-tree", no_argument, 0, 't' },
        { "no-turn-in-place", no_argument, 0, 'N' },
        { "perform-lazy-collision-check", no_argument, 0, 'L' },
        { "help", no_argument, 0, 'h' },
        { 0, 0, 0, 0 }
    };

    while ((c = getopt_long_only (argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c) {
        case 's':
            nom_speed = strtod (optarg, NULL);
            reset_nom_speed = TRUE;
            //self->default_tv = strtod ( optarg , NULL );
            break;
        case 'w':
            check_gridmap_width_buffer = strtod (optarg, NULL);
            fprintf (stdout, "Using %.2f as the additional width buffer for check_gridmap\n", check_gridmap_width_buffer);
            break;
        case 'l':
            sensing_only_local = TRUE;
            fprintf (stdout, "Using only sensing for local gridmap\n");
            break;
        case 'b':
            perform_branch_and_bound = TRUE;
            fprintf (stdout, "Enabling branch-and-bound\n");
            break;
        case 'n':
            no_map = TRUE;
            fprintf (stdout, "Not Using global map\n");
            break;
        case 'D':
            clear_using_laser = FALSE;
            fprintf (stdout, "Clearing using basic method\n");
            break;
        case 'c':
            continuous_map_updates = TRUE;
            //fprintf (stdout, "Using only sensing for local gridmap\n");
            break;
        case 'f':
            large_failsafe = TRUE;
            //fprintf (stdout, "Using only sensing for local gridmap\n");
            break;
        case 't':
            //this will trash the tree at end of every waypoint 
            trash_tree_on_wp = TRUE;
            fprintf (stdout, "Trashing tree on waypoint\n");
            break;
        case 'v':
            verbose = TRUE;
            break;
        case 'd':
            draw = TRUE;
            fprintf (stdout, "Drawing Path checks\n");
            break;
        case 'N':
            enable_turn_in_place = FALSE;
            fprintf (stdout, "Disabling turn-in-place\n");
            break;
        case 'L':
            perform_lazy_collision_check = TRUE;
            fprintf (stdout, "Enabling lazy collision checks\n");
            break;
        case 'h':
            usage (argc, argv);
            return 1;
        default:
            return 1;
        };
    }

    rrtstar_t *self = rrtstar_create (sensing_only_local,trash_tree_on_wp, verbose, draw, 
                                      clear_using_laser, no_map, check_gridmap_width_buffer);
    
    if (!self) {
        fprintf (stderr, "rrtstar_create failed\n");
        return -1;
    }

    if(large_failsafe){
        self->basic_failsafe = 1;
    }
    self->continuous_map_updates = continuous_map_updates;
    self->enable_turn_in_place = enable_turn_in_place;
    self->perform_lazy_collision_check = perform_lazy_collision_check;
    self->perform_branch_and_bound = perform_branch_and_bound;
    fprintf (stdout, "The RRT* is alive\n");
    
    g_mutex_lock (self->plan_mutex);
    
    self->planning_thread = g_thread_create ((GThreadFunc)on_planning_thread, self, FALSE, NULL);
    
    self->mainloop = g_main_loop_new (NULL, TRUE);
    
    // Connect LCM to the mainloop
    bot_glib_mainloop_attach_lcm (self->lcm);

    g_main_loop_run (self->mainloop);

    return 1;

}
