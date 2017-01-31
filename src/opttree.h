/*
  This file is a part of ``RRT(*)'', an incremental 
  sampling-based optimal motion planning library.
  Copyright (c) 2010 Sertac Karaman <sertac@mit.edu>, Emilio Frazzoli <frazzoli@mit.edu>,

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

#ifndef __OPTTREE_H_
#define __OPTTREE_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <glib.h>

// system  
#include "optsystem.h"

// kd-tree interface
#include "kdtree.h"
typedef struct kdtree kdtree_t;
typedef struct kdres kdres_t;


typedef struct _node_t node_t;
struct _node_t {
    state_t *state;                 // The state at this node
    node_t *parent;                 // Parent node to this node
    GSList *children;               // Children of this node
    gboolean reaches_target;        // True if this node reaches the target
    double distance_from_parent;    // Distance of this node to its parent
    double distance_from_root;      // Distance of this node from the root
    GSList *traj_from_parent;       // A sequence of states that connects this node's parent to this node
    GSList *inputs_from_parent;     // A sequence of inputs when applied starting from the parent state will generate traj_from_parent
};


typedef struct _bounding_box_t {
    double *min;
    double *length;
} bounding_box_t;


typedef struct _opttree_t opttree_t;
struct _opttree_t {

    // Dynamical system related parameters
    optsystem_t *optsys;    

    // k-d tree
    kdtree_t *kdtree;

    // Tree structure variables
    node_t *root;                 // Pointer to the root node 
    GSList *list_nodes;           // A list of the nodes in the tree (maintained as a single linked list)
    int num_nodes;                // Number of nodes in the tree (maintained for fast evaluation of the number of nodes)

    // Inputs -- runtime parameters to the algorithm
    gboolean run_rrtstar;         // RRT/RRT* switch; RRT* iteration is executed if this variable is one
    double ball_radius_constant;  // ball shrink rate constant
    double ball_radius_max;       // maximum radius of the shrinking ball
    double target_sample_prob_before_first_solution;    // Probability of sampling the target before the first feasible solution
    double target_sample_prob_after_first_solution;     // Probability of sampling the target after the first feasible solution
 
    // Outputs -- informative values and statistics maintained by the algorithm
    double ball_radius_last;      // Radius of the ball in the last RRT* iteration
    double lower_bound;           // Cost of the best path in the tree
    node_t *lower_bound_node;     // Pointer to the node with lowest cost inside the target region
  
    node_t *lower_bound_node_to_first_goal;     // Pointer to the node with lowest cost inside the target region
};


// Creates a new opttree_t structure and returns a pointer to it
opttree_t* opttree_create (gboolean sensing_only_local, gboolean draw, gboolean clear_using_laser, gboolean sensing_only_small);

// Deallocates memory allocated for a given opttree_t structure
int opttree_destroy (opttree_t *self);

// Sets the root state of the opttree structure
int opttree_set_root_state (opttree_t *self, state_t *state);

// Evaluates a single iteration of the RRT/RRT*
int opttree_iteration (opttree_t *self);

// Empties the tree and reinializes all the related variables
int opttree_reinitialize (opttree_t *self);

// Empties the tree and reinializes all the related variables
int opttree_reset (opttree_t *self);

int opttree_branch_and_bound (opttree_t *self);

GSList* opttree_commit_traj_all (opttree_t *self);

GSList* opttree_commit_traj (opttree_t *self, double time, int *all_committed);

int opttree_get_commit_end_point_to_old_goal (opttree_t *self, double time, 
                                              double *new_root_node);

int opttree_get_commit_end_point (opttree_t *self, double time, 
                                  double *new_root_node);

GSList* opttree_commit_traj_to_old_goal (opttree_t *self, double time, int *all_committed);

void opttree_reset_old_goal_node(opttree_t *self);
#endif 
