/*
  This file is a part of ``RRT(*)'', an incremental 
  sampling-based optimal motion planning library.
  Copyright (c) 2010 Sertac Karaman <sertac@mit.edu>, Emilio Frazzoli <frazzoli@mit.edu>

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
*/

/*
  RRT* algorithm is explained in the following paper:
  http://arxiv.org/abs/1005.0416
*/

#include "optsystem.h"

#define ALPHA 0.5

#define CONSIDER_FOOTPRINT 1 
#define CONSIDER_OBS_COST 0

#define COLLISION_OBS_MAX_THRESHOLD 100

//#define VISUALIZE_CHECK_GRIDMAP TRUE //FALSE

// Number of intermediate states between nodes. Nodes will be added to
// inter-node trajectories to achieve this number of states.
#define NUM_INTERMEDIATE_NODE_STATES 25
#define DISTANCE_LIMIT 100

#define DELTA_DISTANCE .25
#define TURNING_RADIUS 0.5
#define DELTA_T 0.2             // The time interval of integration and node placement


gboolean optsystem_on_obstacle (optsystem_t *self, state_t *state);



int optsystem_extend_with_optimal_control (optsystem_t *self, state_t *state_ini, state_t *state_fin, 
                                           GSList **states_all_out, GSList **inputs_all_out);


double optsystem_extend_dubins (optsystem_t *self, state_t *state_ini, state_t *state_fin, 
                                int *fully_extends, GSList **states_out, GSList **inputs_out);


// Allocates memory for and initializes an empty dynamical system
int optsystem_new_system (optsystem_t *self, gboolean sensing_only_local, gboolean draw, 
                          gboolean clear_using_laser, gboolean sensing_only_small, double check_gridmap_width_buffer) {

    self->initial_state = (state_t *) malloc (sizeof(state_t));
    for (int i = 0; i < NUM_STATES; i++)
        self->initial_state->x[i] = 0.0;

    if(clear_using_laser){
        self->grid = check_gridmap_create_laser(0, draw, sensing_only_local, sensing_only_small, TRUE, FALSE, FALSE, TRUE, check_gridmap_width_buffer);
    }
    else{
        self->grid = check_gridmap_create(0, draw, sensing_only_local, sensing_only_small, TRUE, FALSE, FALSE, check_gridmap_width_buffer);
    }

    self->is_elevator = FALSE;
    return 1;
}


// Frees up the memory used by optsystem_t *self
int optsystem_free_system (optsystem_t *self) {

    if (self->initial_state)
        optsystem_free_state (self, self->initial_state);
    
    check_gridmap_destroy (self->grid);

    return 1;    
}


// Allocates memory for a new state
state_t* optsystem_new_state (optsystem_t *self) {
    state_t *state = (state_t *) malloc (sizeof (state_t)); 
    for (int i = 0; i < NUM_STATES; i++)
        state->x[i] = 0.0;
    return state;
}


// Frees up the memory used by the state
int optsystem_free_state (optsystem_t *self, state_t *state) {
    free (state);
    return 1;
}


// Allocates memory for a new input
input_t *optsystem_new_input (optsystem_t *self) {
    input_t *input = (input_t *) malloc (sizeof (input_t));
    for (int i = 0; i < NUM_INPUTS; i++) 
        input->x[i] = 0.0;
    return input;
}


// Frees up the memory used by the input
int optsystem_free_input (optsystem_t *self, input_t *input) {
    free (input);
    return 1;
}


// Create a copy of the given state
state_t *optsystem_clone_state (optsystem_t *self, state_t *state) {
    state_t *stateNew = (state_t *) malloc (sizeof (state_t));
    for (int i = 0; i < NUM_STATES; i++)
        stateNew->x[i] = state->x[i];
    return stateNew;
}


// Create a copy of the given input
input_t *optsystem_clone_input (optsystem_t *self, input_t *input) {
    input_t *inputNew = (input_t *) malloc (sizeof (input_t));
    for (int i = 0; i < NUM_INPUTS; i++)
        inputNew->x[i] = input->x[i];
    return inputNew;
}


// Sets the initial state to a particular value
int optsystem_set_initial_state (optsystem_t *self, state_t *state) {
    
    for (int i = 0; i < NUM_STATES; i++)
        self->initial_state->x[i] = state->x[i];
    return 1;
}


// Returns the initial state of the system
int  optsystem_get_initial_state (optsystem_t *self, state_t *state) {
    for (int i = 0; i < NUM_STATES; i++) 
        state->x[i] = self->initial_state->x[i];
    return 1;
}


// Returns the ``number of dimensions''
int optsystem_get_num_states (optsystem_t *self) {
    return NUM_STATES;
}


// Returns a double array (of size optsystem_get_num_states) 
//  used for storing the state in a spatial data structure
double* optsystem_get_state_key (optsystem_t *self, state_t *state) {
    return  state->x;
}


// creates a random state  -- dubins car
int optsystem_sample_state (optsystem_t *self, state_t *random_state) { 
    for (int i = 0; i < 2; i++) {
        random_state->x[i] = self->operating_region.size[i]*rand()/(RAND_MAX + 1.0)
            + self->operating_region.center[i] - self->operating_region.size[i]/2.0;
    }
    random_state->x[2] =  2* M_PI * rand()/(RAND_MAX + 1.0);
    if ( optsystem_on_obstacle (self, random_state) ) 
        return 0;    
    return 1;
}


// creates a random state near the goal  -- dubins car
int optsystem_sample_target_state (optsystem_t *self, state_t *random_state) { 
    for (int i = 0; i < 3; i++) {
        random_state->x[i] = self->goal_region.size[i]*rand()/(RAND_MAX + 1.0)
            + self->goal_region.center[i] - self->goal_region.size[i]/2.0;
    }
    //random_state->x[2] =  2* M_PI * rand()/(RAND_MAX + 1.0);
    if ( optsystem_on_obstacle (self, random_state) ) 
        return 0;    
    return 1;
}


// Evaluates the Euclidean distance between two states -- used mainly for the Nearest and CloseNodes procedures
double optsystem_evaluate_distance (optsystem_t *self, state_t *state_from, state_t *state_to) {
    
    double dist = 0;
    for (int i = 0; i < NUM_STATES; i++) {
        double dist_this = state_to->x[i] - state_from->x[i];
        dist += dist_this * dist_this;
    }

    return sqrt (dist);
}


double optsystem_evaluate_distance_for_cost (optsystem_t *self, GSList *inputs, double obstacle_cost) {

    double time = 0.0;

    GSList *inputs_ptr = inputs;
    while (inputs_ptr) {
        input_t *input_curr = inputs_ptr->data;
        time += input_curr->x[NUM_INPUTS-1];
        inputs_ptr = g_slist_next (inputs_ptr);
    }
  
    double alpha_cost = (time * ALPHA /14.38  + (1.0 - ALPHA) * obstacle_cost/ 4.245797);
    
    /*if(self->max_time < time){
      self->max_time = time;
      }

      if(self->max_obs < obstacle_cost){
      self->max_obs = obstacle_cost;
      }*/

    //fprintf(stderr, "Time cost : %.2f Obst : %f\n", self->max_time , self->max_obs);

    if(obstacle_cost > 0){
        //fprintf(stderr, "Time cost : %.2f Obst : %f Comb : %f\n", time, obstacle_cost, alpha_cost); 
    }
#if CONSIDER_OBS_COST
    return alpha_cost;
#endif
    return time;
}

int optsystem_segment_on_obstacle (optsystem_t *self, state_t *state_initial, 
                                   state_t *state_final, int num_steps, double *obstacle_cost) {

    check_gridmap_update(self->grid);

    //Check path
    int is_forward = 1;
    int failsafe = 0;

    if(self->failsafe_level){
        //use increased failsafe
        failsafe = self->failsafe_level; 
    }

    //if there is a faliure also - we might increase the failsafe ??
    
    // Increase the failsafe for elevators
    if (self->is_elevator == TRUE)
        failsafe = 2;

    struct check_path_result path_res;
    
    check_gridmap_check_path (self->grid, is_forward, failsafe,
                              state_initial->x[0], state_initial->x[1], state_initial->x[2],
                              state_final->x[0], state_final->x[1], state_final->x[2],
                              &path_res);

    //*obstacle_cost = path_res.cost;
    //double distance = sqrt(bot_sq(state_initial->x[0]-state_final->x[0]) + bot_sq(state_initial->x[1]-state_final->x[1]));

    // We consider the segment to be in collision if either
    //   (i)  The line-integrated cost is negative or
    //   (ii) The maximum obstacle cost along the path exceeds a threshold
    if ((path_res.cost < 0) || (path_res.obs_max > COLLISION_OBS_MAX_THRESHOLD) || (path_res.obs_max) < 0) {
        *obstacle_cost = DBL_MAX;
        return 1;
    }
    else 
        *obstacle_cost = path_res.cost / 254;
        
    return 0;

    /* 
     * //cost / (worst cost * steps)
     * if(path_res.obs_max == 0){
     *     *obstacle_cost = path_res.cost / 254;    
     *     //\*obstacle_cost = path_res.obs_max/254;  //cost/ 254;
     * }
     * else{
     *     *obstacle_cost = path_res.cost / 254;//(path_res.obs_max);// * self->grid->convolve_line_searches); 
     *     //\*obstacle_cost = path_res.obs_max/254;  //cost / 254;//(path_res.obs_max);// * self->grid->convolve_line_searches); 
     * }
     * 
     * if(path_res.obs_max != 0){
     *     //fprintf(stderr, "Dist : %f Path: %f, Max : %f , Conv : %d Obstacle Cost : %f \n", distance, path_res.cost, path_res.obs_max, self->grid->convolve_line_searches, *obstacle_cost);
     * }
     * if (path_res.obs_max > 100 /\*for spread cost 200*\/ /\*10*\/ || path_res.obs_max < 0 ){ //was 10
     * 
     *     *obstacle_cost = DBL_MAX;
     *     //Debug prints
     * 
     *     //printf("seg path_res:\n cost %f\n restricted %f\n max %f\n obs_cost %f\n obs_restricted %f\n obs_max %f\n", path_res.cost, path_res.restricted, path_res.max, path_res.obs_cost, path_res.obs_restricted, path_res.obs_max);
     * 
     *     return 1;    
     * }
     * else
     *     return 0;
     */
}
/*    GSList *obstacle_list_curr = self->obstacle_list;

//AABB check

while (obstacle_list_curr) {
region_2d_t *obstacle_region = (region_2d_t *) (obstacle_list_curr->data);
double center[2] = {
obstacle_region->center[0],
obstacle_region->center[1]
};
double size[2] = {
obstacle_region->size[0]/2,
obstacle_region->size[1]/2
};

double disc[2] = {
(state_final->x[0] - state_initial->x[0])/( (double)num_steps ),
(state_final->x[1] - state_initial->x[1])/( (double)num_steps )
};

double state_curr[3] = {
state_initial->x[0],
state_initial->x[1],
state_initial->x[2]
};

if (!CONSIDER_FOOTPRINT){

for (int i = 0; i < num_steps; i++) {
if ( (fabs(center[0] - state_curr[0]) <= size[0]) && (fabs(center[1] - state_curr[1]) <= size[1]) ) 
return 1;

state_curr[0] += disc[0];
state_curr[1] += disc[1];
}

}
else{

//Forklift values obtained from agile.cfg    
//use from param 
double width = 0.6;//4.00 / 2;
double length = 1.0; //1.70 / 2;

//Separating Axis Theorem

for (int i = 0; i < num_steps; i++) {

double c_x = state_curr[0];
double c_y = state_curr[1];

// Move forklift rect to make obstacle rect cannonic
c_x -= center[0];
c_y -= center[1];

// Rotate forklift rect, make it axis-aligned 
double cosa = cos(state_curr[2] * (180.0/M_PI));
double sina = sin(state_curr[2] * (180.0/M_PI));

double t = c_x;
c_x = t * cosa - c_y * sina;
c_y = t * sina + c_y * cosa;


// Calculate forklift vertices
double bl_x, tr_x, bl_y, tr_y;

bl_x = tr_x = c_x;
bl_y = tr_y = c_y;

bl_x -= width;
bl_y -= length;

tr_x += width;
tr_y += length;

// Calculate obstacle vertices
double A_x = -size[1] * sina;
double B_x = A_x;
t = size[0] * cosa;
A_x += t;
B_x -= t;

double A_y = size[1] * cosa;
double B_y = A_y;
t = size[0] * sina;
A_y += t;
B_y -= t;

t = sina * cosa;

// Check if A is vertical min/max,
// B is horizontal min/max
if (t < 0)
{
t = A_x; A_x = B_x; B_x = t;
t = A_y; A_y = B_y; B_y = t;
}

// Check if B is horizontal minimum 
if (sina < 0) { B_x = -B_x; B_y = -B_y; }

// If forklift is not in the horizontal range
// there is no collision
if ( B_x > tr_x || B_x > -bl_x ) break;
double ext1, ext2, x, a, dx;

// If obstacle is axis-alligned, get vertical min/max
if (t == 0) {
ext1 = A_y;
ext2 = -ext1; 
}
// Otherwise, find vertical min/max in the range bottom left, top right
else
{
x = bl_x - A_x; 
a = tr_x - A_x;
ext1 = A_y;

if (a*x > 0)
{
dx = A_x;
if (x < 0) { dx -= B_x; ext1 -= B_y; x = a; }
else       { dx += B_x; ext1 += B_y; }
ext1 *= x; ext1 /= dx; ext1 += A_y;
}

x = bl_x + A_x; a = tr_x + A_x;
ext2 = -A_y;


if (a*x > 0)
{
dx = -A_x;
if (x < 0) { dx -= B_x; ext2 -= B_y; x = a; }
else       { dx += B_x; ext2 += B_y; }
ext2 *= x; ext2 /= dx; ext2 -= A_y;
}
}

// Collision check 
if ( ! ((ext1 < bl_y && ext2 < bl_y) ||
(ext1 > tr_y && ext2 > tr_y)) )
return 1;

state_curr[2] = atan2( ( ( state_curr[1] + disc[1] ) - (state_curr[1]) ) ,
( state_curr[0] + disc[0] ) - (state_curr[0]) );

state_curr[0] += disc[0];
state_curr[1] += disc[1];

}

}

obstacle_list_curr = g_slist_next (obstacle_list_curr);
}

return 0;
}
*/
int optsystem_state_out_of_operating_region (optsystem_t *self, state_t *state_curr) {
    
    if  ( ( fabs(state_curr->x[0] - self->operating_region.center[0]) > self->operating_region.size[0]/2.0 ) 
         || ( fabs(state_curr->x[1] - self->operating_region.center[1]) > self->operating_region.size[1]/2.0 ) ) 
        return 1;
    
    return 0;

}

// Extends a given state towards another state - dubins car
int optsystem_extend_to (optsystem_t *self, state_t *state_from, state_t *state_towards, 
                         int *fully_extends, GSList **states_all_out,
                         int *num_node_states, int **node_states, GSList **inputs_all_out, double *obstacle_cost_out,
                         gboolean stop_at_goal) {

    int discretization_num_steps = 20;

    int num_intermediate_traj = NUM_INTERMEDIATE_NODE_STATES;

    GSList *states_all = NULL;
    GSList *inputs_all = NULL;

    // Compute the optimal control 
    if (optsystem_extend_dubins (self, state_from, state_towards, fully_extends, &states_all, &inputs_all) == -1)
        return 0;

    // Check states for collision avoidance
    int num_states = 0;   // Count the number of states at the same time
    state_t *state_prev = state_from;
    GSList *states_ptr = states_all; 
    GSList *inputs_ptr = inputs_all;

    while (states_ptr && inputs_ptr) {
        num_states++;
        state_t *state_curr = (state_t *)(states_ptr->data);
        double obstacle_cost = 0;
        if (optsystem_segment_on_obstacle (self, state_prev, state_curr, discretization_num_steps, &obstacle_cost)  || 
            optsystem_state_out_of_operating_region (self, state_curr) ){
            // Free the states/inputs
            GSList *states_tmp = states_all;
            while (states_tmp) {
                optsystem_free_state (self, (state_t *)(states_tmp->data));
                states_tmp = g_slist_next (states_tmp);
            }
            g_slist_free (states_all);
            GSList *inputs_tmp = inputs_all;
            while (inputs_tmp) {
                optsystem_free_input (self, (input_t *)(inputs_tmp->data));
                inputs_tmp = g_slist_next (inputs_tmp);
            }
            g_slist_free (inputs_all);
            *fully_extends = 0;
            *states_all_out = NULL;
            *num_node_states = 0;
            *node_states = NULL;
            *inputs_all_out = NULL;
            return 0;
        }
        *obstacle_cost_out += obstacle_cost;
        state_prev = state_curr;
        states_ptr = g_slist_next (states_ptr);
        inputs_ptr = g_slist_next (inputs_ptr);

        // Check whether this state happens to be at the goal
        if (stop_at_goal && states_ptr && (optsystem_is_reaching_target (self, state_curr) == 1)) {

            // Free remaining states and inputs
            while (states_ptr) {
                state_t *state_next = (state_t *) (states_ptr->data);
                states_all = g_slist_remove (states_all, state_next);
                optsystem_free_state (self, state_next);
                states_ptr = g_slist_next (states_ptr);
            }

            while (inputs_ptr) {
                input_t *input_next = (input_t *) (inputs_ptr->data);
                inputs_all = g_slist_remove (inputs_all, input_next);
                optsystem_free_input (self, input_next);
                inputs_ptr = g_slist_next (inputs_ptr);
            }

            *fully_extends = 0;
            break;
        }
    }

    // Assign the intermediate nodes solely with the number
    // TODO: do this time-based
    int num_node_states_tmp = (int) floor ( ((double)num_states)/((double)num_intermediate_traj + 1));
    

    if (num_node_states_tmp > 0) {
        int *node_states_tmp = (int *) malloc (num_node_states_tmp * sizeof (int));
        for (int i = 0; i < num_node_states_tmp; i++) 
            node_states_tmp[i] = (i+1) * (num_intermediate_traj+1) - 1; 
        *num_node_states = num_node_states_tmp;
        *node_states = node_states_tmp;
    }
    else {
        *num_node_states = 0;
        *node_states = NULL;
    }
    *states_all_out = states_all;
    *inputs_all_out = inputs_all;
    
    return 1;
}

gboolean optsystem_on_obstacle (optsystem_t *self, state_t *state) {

    check_gridmap_update(self->grid);

    // Check whether the state is in collision by forming a simple path
    double dpos = 0.1;
    double x1 = state->x[0] - dpos/2*cos(state->x[2]);
    double x2 = state->x[0] + dpos/2*cos(state->x[2]);
    double y1 = state->x[1] - dpos/2*sin(state->x[2]);
    double y2 = state->x[1] + dpos/2*sin(state->x[2]);

    int is_forward = 1;
    int failsafe = 0;
    struct check_path_result path_res;

    check_gridmap_check_path (self->grid, is_forward, failsafe,
                              x1, y1, state->x[2],
                              x2, y2, state->x[2],
                              &path_res);

    /*fprintf(stderr, "Path Cost : %f, Path Max Cost : %f\n", 
                path_res.cost, 
                path_res.max);
    */

    // Use the maximum cost to determine whether the sample is in collision
    if (path_res.max > 50 || path_res.max < 0 || path_res.cost <0){
        //fprintf(stderr,"path_res:\n cost %f\n restricted %f\n max %f\n obs_cost %f\n obs_restricted %f\n obs_max %f\n", path_res.cost, path_res.restricted, path_res.max, path_res.obs_cost, path_res.obs_restricted, path_res.obs_max);

        return 1;    
    }
    else{
        //fprintf(stderr,"Neg cost\n");
        return 0;
    }
}

/*
//AABB check

GSList *obstacle_list_curr = self->obstacle_list;

double width = 4.00 / 2;
double length = 1.70 / 2;

while (obstacle_list_curr) {
region_2d_t *obstacle_region = (region_2d_t *) (obstacle_list_curr->data);

if (CONSIDER_FOOTPRINT){

double c_x = state->x[0];
double c_y = state->x[1];

c_x -= obstacle_region->center[0];
c_y -= obstacle_region->center[1];


double cosa = cos(state->x[2] * (180.0/M_PI));
double sina = sin(state->x[2] * (180.0/M_PI));

double t = c_x;
c_x = t * cosa - c_y * sina;
c_y = t * sina + c_y * cosa;

double bl_x, tr_x, bl_y, tr_y;

bl_x = tr_x = c_x;
bl_y = tr_y = c_y;

bl_x -= width;
bl_y -= length;

tr_x += width;
tr_y += length;

double A_x = -( obstacle_region->size[1]/2.0 ) * sina;
double B_x = A_x;
t = ( obstacle_region->size[0]/2.0 ) * cosa;
A_x += t;
B_x -= t;

double A_y = ( obstacle_region->size[1]/2.0 ) * cosa;
double B_y = A_y;
t = ( obstacle_region->size[0]/2.0 ) * sina;
A_y += t;
B_y -= t;

t = sina * cosa;

if (t < 0)
{
t = A_x; A_x = B_x; B_x = t;
t = A_y; A_y = B_y; B_y = t;
}

if (sina < 0) { B_x = -B_x; B_y = -B_y; }

if (B_x > tr_x || B_x > -bl_x) return 0;

double ext1, ext2, x, a, dx;

if (t == 0) {
ext1 = A_y;
ext2 = -ext1; 
}
else
{
x = bl_x - A_x; 
a = tr_x - A_x;
ext1 = A_y;

if (a*x > 0)
{
dx = A_x;
if (x < 0) { dx -= B_x; ext1 -= B_y; x = a; }
else       { dx += B_x; ext1 += B_y; }
ext1 *= x; ext1 /= dx; ext1 += A_y;
}

x = bl_x + A_x; a = tr_x + A_x;
ext2 = -A_y;

if (a*x > 0)
{
dx = -A_x;
if (x < 0) { dx -= B_x; ext2 -= B_y; x = a; }
else       { dx += B_x; ext2 += B_y; }
ext2 *= x; ext2 /= dx; ext2 -= A_y;
}
}

return !((ext1 < bl_y && ext2 < bl_y) ||
(ext1 > tr_y && ext2 > tr_y));
}
else{
if ( (fabs(obstacle_region->center[0] - state->x[0]) <= obstacle_region->size[0]/2.0) &&
(fabs(obstacle_region->center[1] - state->x[1]) <= obstacle_region->size[1]/2.0) ) 
return 1;
}

obstacle_list_curr = g_slist_next (obstacle_list_curr);
}

return 0;
}
*/

gboolean optsystem_is_reaching_target (optsystem_t *self, state_t *state) {
    
    if ( (fabs(self->goal_region.center[0] - state->x[0]) <= self->goal_region.size[0]/2.0) &&
        (fabs(self->goal_region.center[1] - state->x[1]) <= self->goal_region.size[1]/2.0)
        && (fabs(bot_mod2pi(self->goal_region.center[2] - state->x[2])) <= self->goal_region.size[2]/2.0)){ 
        return 1;
    }
    
    return 0;
}


gboolean optsystem_update_obstacles (optsystem_t *self, GSList *obstacle_list) {

    // Clear the previous obstacles
    while (self->obstacle_list) {
        region_2d_t *region_curr = (region_2d_t *) (self->obstacle_list->data);
        self->obstacle_list = g_slist_remove (self->obstacle_list, region_curr);
        free (region_curr);
    }
    
    // Add new obstacles
    GSList *obstacle_list_curr = obstacle_list;
    while (obstacle_list_curr) {
        region_2d_t *region_curr = (region_2d_t *) (obstacle_list_curr->data);
        region_2d_t *region_new = (region_2d_t *) malloc (sizeof (region_2d_t));
        region_new->center[0] = region_curr->center[0];
        region_new->center[1] = region_curr->center[1];
        region_new->center[2] = region_curr->center[2];
        region_new->size[0] = region_curr->size[0];
        region_new->size[1] = region_curr->size[1];
        region_new->size[2] = region_curr->size[2];
        self->obstacle_list = g_slist_prepend (self->obstacle_list, (gpointer)region_new);

        obstacle_list_curr = g_slist_next (obstacle_list_curr);
    }

    return TRUE;
}

gboolean optsystem_update_goal_region (optsystem_t *self, region_3d_t *goal_region) {

    self->goal_region.center[0] = goal_region->center[0];
    self->goal_region.center[1] = goal_region->center[1];
    self->goal_region.center[2] = goal_region->center[2];
    self->goal_region.size[0] = goal_region->size[0];
    self->goal_region.size[1] = goal_region->size[1];
    self->goal_region.size[2] = goal_region->size[2];
    return TRUE;
}

gboolean optsystem_update_goal_type (optsystem_t *self, gboolean is_elevator) {
   
    self->is_elevator = is_elevator;
    return TRUE;
}

gboolean optsystem_update_operating_region (optsystem_t *self, region_2d_t *operating_region) {

    self->operating_region.center[0] = operating_region->center[0];
    self->operating_region.center[1] = operating_region->center[1];
    self->operating_region.size[0] = operating_region->size[0];
    self->operating_region.size[1] = operating_region->size[1];

    return TRUE;
}

int optsystem_extend_dubins_spheres (optsystem_t *self, double x_s1, double y_s1, double t_s1, 
                                     double x_s2, double y_s2, double t_s2, int comb_no,
                                     int *fully_extends, GSList **states_all, GSList **inputs_all) {
    
    double x_tr = x_s2 - x_s1;
    double y_tr = y_s2 - y_s1;
    double t_tr = atan2 (y_tr, x_tr);
    
    double distance = sqrt ( x_tr*x_tr + y_tr*y_tr );

    // The position and orientation
    double x_start;
    double y_start;
    double t_start;
    double x_end;
    double y_end;
    double t_end;
    
    if (distance > 2 * TURNING_RADIUS) {  // disks do not intersect 

        double t_balls = acos (2 * TURNING_RADIUS / distance);
        

        switch (comb_no) {
        case 1:
            t_start = t_tr - t_balls;
            t_end = t_tr + M_PI - t_balls;
            break;
        case 2:
            t_start = t_tr + t_balls;
            t_end = t_tr - M_PI + t_balls;
            break;
        case 3:
            t_start = t_tr - M_PI_2;
            t_end = t_tr - M_PI_2;
            break;
        case 4:
            t_start = t_tr + M_PI_2;
            t_end = t_tr + M_PI_2;
            break;
        default:
            return -1.0;
        }
    }

    else { // disks are intersecting
        
        switch (comb_no) {
        case 1:
        case 2:
            // No solution
            if (states_all) { *states_all = NULL;
                *inputs_all = NULL;
            }
            return -1.0;
            break;
            
        case 3:
            t_start = t_tr - M_PI_2;
            t_end = t_tr - M_PI_2;
            break;
        case 4:
            t_start = t_tr + M_PI_2;
            t_end = t_tr + M_PI_2;
            break;
        }
    }
    
    x_start = x_s1 + TURNING_RADIUS * cos (t_start);
    y_start = y_s1 + TURNING_RADIUS * sin (t_start);
    x_end = x_s2 + TURNING_RADIUS * cos (t_end);
    y_end = y_s2 + TURNING_RADIUS * sin (t_end);

    int direction_s1 = 1;
    if ( (comb_no == 2) || (comb_no == 4) ) {
        direction_s1 = -1;
    }
    int direction_s2 = 1;
    if ( (comb_no == 1) || (comb_no == 4) ) {
        direction_s2 = -1;
    }
    
    double t_increment_s1 = direction_s1 * (t_start - t_s1);
    double t_increment_s2 = direction_s2 * (t_s2 - t_end);

    while (t_increment_s1 < 0) 
        t_increment_s1 += 2.0 * M_PI;
    while (t_increment_s1 > 2.0 *M_PI)
        t_increment_s1 -= 2.0 * M_PI;

    while (t_increment_s2 < 0) 
        t_increment_s2 += 2.0 * M_PI;
    while (t_increment_s2 > 2.0 *M_PI)
        t_increment_s2 -= 2.0 * M_PI;

    if  ( ( (t_increment_s1 > M_PI) && (t_increment_s2 > M_PI) ) 
         || ( (t_increment_s1 > 3*M_PI_2) || (t_increment_s2 > 3*M_PI_2) )  ){
        return -1.0;
    }
    
    double total_distance_travel = (t_increment_s1 + t_increment_s2) * TURNING_RADIUS  + distance;
    
    double distance_limit = DISTANCE_LIMIT;

    if (fully_extends)
        *fully_extends = 0;
    
    if (states_all) {
        // Generate states/inputs
        
        GSList *states = NULL;
        GSList *inputs = NULL;
        
        double del_d = DELTA_DISTANCE;
        double del_t = del_d * TURNING_RADIUS;
        
        double t_inc_curr = 0.0;

        
        while (t_inc_curr < t_increment_s1) {
            double t_inc_rel = del_t;
            t_inc_curr += del_t;
            if (t_inc_curr > t_increment_s1) {
                t_inc_rel -= t_inc_curr - t_increment_s1;
                t_inc_curr = t_increment_s1;
            }
            
            state_t *state_curr = optsystem_new_state (self);
            input_t *input_curr = optsystem_new_input (self);

            state_curr->x[0] = x_s1 + TURNING_RADIUS * cos (direction_s1 * t_inc_curr + t_s1);
            state_curr->x[1] = y_s1 + TURNING_RADIUS * sin (direction_s1 * t_inc_curr + t_s1);
            state_curr->x[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);
            while (state_curr->x[2] < 0)
                state_curr->x[2] += 2 * M_PI;
            while (state_curr->x[2] > 2 * M_PI)
                state_curr->x[2] -= 2 * M_PI;

            input_curr->x[0] = ( (comb_no == 1) || (comb_no == 3) ) ? -1 : 1;
            input_curr->x[1] = t_inc_rel * TURNING_RADIUS;


            states = g_slist_prepend (states, state_curr);
            inputs = g_slist_prepend (inputs, input_curr);

            if (t_inc_curr * TURNING_RADIUS > distance_limit) 
                goto trajectory_complete;
        }
        
        double d_inc_curr = 0.0;
        while (d_inc_curr < distance) {
            double d_inc_rel = del_d;
            d_inc_curr += del_d;
            if (d_inc_curr > distance) {
                d_inc_rel -= d_inc_curr - distance;
                d_inc_curr = distance;
            }
            
            state_t *state_curr = optsystem_new_state (self);
            input_t *input_curr = optsystem_new_input (self);
            
            state_curr->x[0] = (x_end - x_start) * d_inc_curr / distance + x_start; 
            state_curr->x[1] = (y_end - y_start) * d_inc_curr / distance + y_start; 
            state_curr->x[2] = direction_s1 * t_inc_curr + t_s1 + ( (direction_s1 == 1) ? M_PI_2 : 3.0*M_PI_2);
            while (state_curr->x[2] < 0)
                state_curr->x[2] += 2 * M_PI;
            while (state_curr->x[2] > 2 * M_PI)
                state_curr->x[2] -= 2 * M_PI;

            input_curr->x[0] = 0.0;
            input_curr->x[1] = d_inc_rel;

            states = g_slist_prepend (states, state_curr);
            inputs = g_slist_prepend (inputs, input_curr);

            if (t_inc_curr * TURNING_RADIUS + d_inc_curr > distance_limit) 
                goto trajectory_complete;
        }
        
        double t_inc_curr_prev = t_inc_curr;
        t_inc_curr = 0.0;
        while (t_inc_curr < t_increment_s2) {
            double t_inc_rel = del_t;
            t_inc_curr += del_t;
            if (t_inc_curr > t_increment_s2)  {
                t_inc_rel -= t_inc_curr - t_increment_s2;
                t_inc_curr = t_increment_s2;
            }
            
            state_t *state_curr = optsystem_new_state (self);
            input_t *input_curr = optsystem_new_input (self);

            state_curr->x[0] = x_s2 + TURNING_RADIUS * cos (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
            state_curr->x[1] = y_s2 + TURNING_RADIUS * sin (direction_s2 * (t_inc_curr - t_increment_s2) + t_s2);
            state_curr->x[2] = direction_s2 * (t_inc_curr - t_increment_s2) + t_s2 
                + ( (direction_s2 == 1) ?  M_PI_2 : 3.0*M_PI_2 );
            while (state_curr->x[2] < 0)
                state_curr->x[2] += 2 * M_PI;
            while (state_curr->x[2] > 2 * M_PI)
                state_curr->x[2] -= 2 * M_PI;
            
            input_curr->x[0] = ( (comb_no == 2) || (comb_no == 3) ) ? -1 : 1;
            input_curr->x[1] = t_inc_rel * TURNING_RADIUS;

            states = g_slist_prepend (states, state_curr);
            inputs = g_slist_prepend (inputs, input_curr);

            if ((t_inc_curr_prev + t_inc_curr) * TURNING_RADIUS + d_inc_curr > distance_limit)
                goto trajectory_complete;
        }

        if (fully_extends)
            *fully_extends = 1;

    trajectory_complete:

        *states_all = g_slist_reverse (states);
        *inputs_all = g_slist_reverse (inputs);
    }
    
    return total_distance_travel;
}


double optsystem_extend_dubins (optsystem_t *self, state_t *state_ini, state_t *state_fin, 
                                int *fully_extends, GSList **states_out, GSList **inputs_out) {
    
    
    // 1. Compute the centers of all four spheres
    double ti = state_ini->x[2];
    double tf = state_fin->x[2];
    double sin_ti = sin (-ti);
    double cos_ti = cos (-ti);
    double sin_tf = sin (-tf);
    double cos_tf = cos (-tf);
    
    double si_left[3] = {
        state_ini->x[0] + TURNING_RADIUS * sin_ti,
        state_ini->x[1] + TURNING_RADIUS * cos_ti,
        ti + 3 * M_PI_2
    };
    double si_right[3] = {
        state_ini->x[0] - TURNING_RADIUS * sin_ti,
        state_ini->x[1] - TURNING_RADIUS * cos_ti,
        ti + M_PI_2
    };
    double sf_left[3] = {
        state_fin->x[0] + TURNING_RADIUS * sin_tf,
        state_fin->x[1] + TURNING_RADIUS * cos_tf,
        tf + 3 * M_PI_2
    };
    double sf_right[3] = {
        state_fin->x[0] - TURNING_RADIUS * sin_tf,
        state_fin->x[1] - TURNING_RADIUS * cos_tf,
        tf + M_PI_2
    };
    
    // 2. extend all four spheres
    double times[4]; 
    
    times[0] = optsystem_extend_dubins_spheres (self, si_left[0], si_left[1], si_left[2], 
                                                sf_right[0], sf_right[1], sf_right[2], 1,
                                                NULL, NULL, NULL);
    times[1] = optsystem_extend_dubins_spheres (self, si_right[0], si_right[1], si_right[2], 
                                                sf_left[0], sf_left[1], sf_left[2], 2,
                                                NULL, NULL, NULL );
    times[2] = optsystem_extend_dubins_spheres (self, si_left[0], si_left[1], si_left[2], 
                                                sf_left[0], sf_left[1], sf_left[2], 3,
                                                NULL, NULL, NULL );
    times[3] = optsystem_extend_dubins_spheres (self, si_right[0], si_right[1], si_right[2], 
                                                sf_right[0], sf_right[1], sf_right[2], 4,
                                                NULL, NULL, NULL );

    double min_time = DBL_MAX;
    int comb_min = -1;
    for (int i = 0; i < 4; i++) {
        if  ( (times[i] >= 0.0) && (times[i] < min_time) ) {
            comb_min = i+1;
            min_time = times[i];
        }
    }
    
    int res;
    switch (comb_min) {
    case 1:
        res = optsystem_extend_dubins_spheres (self, si_left[0], si_left[1], si_left[2], 
                                               sf_right[0], sf_right[1], sf_right[2], 1,
                                               fully_extends, states_out, inputs_out);
        return res;
        
    case 2:
        res = optsystem_extend_dubins_spheres (self, si_right[0], si_right[1], si_right[2], 
                                               sf_left[0], sf_left[1], sf_left[2], 2,
                                               fully_extends, states_out, inputs_out);
        return res;

    case 3:
        res = optsystem_extend_dubins_spheres (self, si_left[0], si_left[1], si_left[2], 
                                               sf_left[0], sf_left[1], sf_left[2], 3,
                                               fully_extends, states_out, inputs_out);
        return res;

    case 4:
        res = optsystem_extend_dubins_spheres (self, si_right[0], si_right[1], si_right[2], 
                                               sf_right[0], sf_right[1], sf_right[2], 4,
                                               fully_extends, states_out, inputs_out);
        return res;
    case -1:
    default:
        if (states_out) {
            *states_out = NULL;
            *inputs_out = NULL;
        }
        return -1.0;
    }
}

double optsystem_evaluate_cost_to_go (optsystem_t *self, state_t *state) {

    // TODO: This function should calculate the the cost to go from state to the goal region, 
    //       by integrating the optimal control towards the goal region. 

    // If state is in the goal region then return zero
    if (optsystem_is_reaching_target (self, state)) 
        return 0.0;

    // Otherwise calculate a lower bound on the cost to go 
    // TODO: do the exact cost to go calculation

    double min_side = self->goal_region.size[0]/2.0;
    if (self->goal_region.size[1] < min_side)
        min_side = self->goal_region.size[1]/2.0;

    double dist_x = state->x[0] - self->goal_region.center[0];
    double dist_y = state->x[1] - self->goal_region.center[1];

    dist_x = fabs(dist_x);
    dist_y = fabs(dist_y);
    
    double dist = sqrt(dist_x*dist_x + dist_y*dist_y);
    if (dist_x < dist_y) {
        dist -= sqrt(1 + (dist_x/dist_y)*(dist_x/dist_y))*self->goal_region.size[0]/2.0;
    }
    else {
        dist -= sqrt(1 + (dist_y/dist_x)*(dist_y/dist_x))*self->goal_region.size[0]/2.0;
    }

    dist-= 0.5;
    if (dist < 0.0)
        dist = 0.0;
   
    //******** This is what causes the memory leak - esentially ignoring the returned extend_dubins method *****////
    
    //calculate time with the extend function

    /*int fully_extends = 0;
    GSList *trajectory = NULL;
    int num_node_states = 0;
    int  *node_states = NULL;
    GSList *inputs = NULL;
    double obstacle_cost = 0;
    
    state_t state_towards;
    state_towards.x[0] = self->goal_region.center[0];
    state_towards.x[1] = self->goal_region.center[1];
    state_towards.x[2] = self->goal_region.center[2];
    
    // Check to see whether a path exists without considering obstacles and populate the input
    if (optsystem_extend_dubins (self, state, &state_towards, &fully_extends, &trajectory, &inputs) == -1)
        return DBL_MAX;

    double time = 0.0;

    GSList *inputs_ptr = inputs;
    while (inputs_ptr) {
        input_t *input_curr = inputs_ptr->data;
        time += input_curr->x[NUM_INPUTS-1];
        inputs_ptr = g_slist_next (inputs_ptr);
    }

    //assume no obstacles and use same cost metric
#if CONSIDER_OBS_COST
    return (time/14.38) * ALPHA + 0.0; 
#endif
return time;*/
    return dist;
}
