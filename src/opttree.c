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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>

#include <glib.h>
#include <bot_core/bot_core.h>

#include "opttree.h"

#define BNB_TOLERANCE 1.0

int opttree_check_if_solution_in_decendants (opttree_t *self, GSList *node_list, node_t *node_check);

// Allocate memory for a new node
node_t* opttree_new_node (opttree_t *self) {
    node_t *node_new = (node_t *) malloc (sizeof (node_t));
    node_new->parent = NULL;
    node_new->children = NULL;
    node_new->state = optsystem_new_state (self->optsys); 
    node_new->reaches_target = FALSE;
    node_new->distance_from_root = 0.0;
    node_new->distance_from_parent = 0.0;
    node_new->traj_from_parent = NULL;
    node_new->inputs_from_parent = NULL;
    return node_new;
}


// Allocate memory for a new node, but does not allocate memory for the state
node_t* opttree_new_node_no_state () {
    node_t *node_new = (node_t *) malloc (sizeof (node_t));
    node_new->parent = NULL;
    node_new->children = NULL;
    node_new->state = NULL;
    node_new->reaches_target = FALSE;
    node_new->distance_from_root = 0.0;
    node_new->distance_from_parent = 0.0;
    node_new->traj_from_parent = NULL;
    node_new->inputs_from_parent = NULL;
    return node_new;
}


// Free node 
int opttree_free_node (opttree_t *self, node_t *node) {
    
    optsystem_free_state (self->optsys, node->state);
    g_slist_free (node->children);
    node->children = NULL;
    
    GSList *traj_ptr = node->traj_from_parent;
    while (traj_ptr) {
        optsystem_free_state (self->optsys, (state_t *)(traj_ptr->data));
        traj_ptr = g_slist_next (traj_ptr);
    }
    g_slist_free (node->traj_from_parent);

    GSList *inputs_ptr = node->inputs_from_parent;
    while (inputs_ptr) {
        optsystem_free_input (self->optsys, (input_t*)(inputs_ptr->data));
        inputs_ptr = g_slist_next (inputs_ptr);
    }
    g_slist_free (node->inputs_from_parent);

    free (node);
    
    return 1;
}


// Free node 
int opttree_free_node_not_state (opttree_t *self, node_t *node) {
    
    //     optsystem_free_state (self->optsys, node->state);
    g_slist_free (node->children);
    node->children = NULL;
    
    GSList *traj_ptr = node->traj_from_parent;
    while (traj_ptr) {
        optsystem_free_state (self->optsys, (state_t *)(traj_ptr->data));
        traj_ptr = g_slist_next (traj_ptr);
    }
    g_slist_free (node->traj_from_parent);

    GSList *inputs_ptr = node->inputs_from_parent;
    while (inputs_ptr) {
        optsystem_free_input (self->optsys, (input_t*)(inputs_ptr->data));
        inputs_ptr = g_slist_next (inputs_ptr);
    }
    g_slist_free (node->inputs_from_parent);

    free (node);
    
    return 1;
}


// Find the nearest neighbor in the tree
node_t* opttree_find_nearest_neighbor (opttree_t *self, state_t *state_from) {

    node_t *min_node = NULL;

    kdres_t *kdres = kd_nearest (self->kdtree, optsystem_get_state_key (self->optsys, state_from));
    if (!kdres) {
        fprintf (stderr, "ERROR: Nearest neighbor from KD tree is NULL\n");
        exit(1);
    }

    if (kd_res_end (kdres))  {
        printf ("ERROR: No nearest neighbors\n");
        exit(1);
    }
    min_node = kd_res_item_data (kdres);
    kd_res_free (kdres);
    
    return min_node;
}


// Recursively update the distances
void opttree_update_distance_from_root (opttree_t *self, node_t *node_parent, node_t *node_curr) {

    node_curr->distance_from_root = node_parent->distance_from_root + node_curr->distance_from_parent;

    // Check for reachability of the target, if so update the lower_bound
    if (optsystem_is_reaching_target (self->optsys, node_curr->state)) {
        if (node_curr->distance_from_root < self->lower_bound) {
            self->lower_bound = node_curr->distance_from_root;
            self->lower_bound_node = node_curr;
        }
        node_curr->reaches_target = 1;
    }
    else 
        node_curr->reaches_target = 0;
    
    GSList *node_child_list = node_curr->children;
    while (node_child_list){
        opttree_update_distance_from_root (self, node_curr, (node_t *)(node_child_list->data));
        node_child_list = g_slist_next (node_child_list);
    }
}


// Adds a given trajectory to the graph and returns a pointer to the the last node
node_t *opttree_add_traj_to_graph (opttree_t *self, node_t *node_start, node_t *node_end, 
                                   GSList *trajectory, int num_node_states, int *node_states, GSList *inputs, double obstacle_cost) {

    node_t *node_prev = node_start;   // This variable maintains the node last added to the graph to update parents

    int trajectory_count = 0;
    int node_states_count = 0;

    GSList *subtraj_curr = NULL;
    GSList *inputs_curr = NULL;

    GSList *inputs_ptr = inputs; 
    GSList *trajectory_ptr = trajectory;

    while (trajectory_ptr) {

        state_t *state_curr = (state_t *) (trajectory_ptr->data);

        // Check whether this the end of the trajectory
        if (!g_slist_next (trajectory_ptr)) {
            // This must be the last node in the traj.

            node_t *node_curr;
            if (node_end) {    // If node_end is given, then modify node end accordingly
                
                node_curr = node_end;
                node_t *node_parent = node_curr->parent;
                node_parent->children = g_slist_remove (node_parent->children, (gpointer) node_curr);
                // Free traj_from_parent
                GSList *traj_from_parent_ptr = node_curr->traj_from_parent;
                while (traj_from_parent_ptr) {
                    optsystem_free_state (self->optsys, (state_t *) (traj_from_parent_ptr->data));
                    traj_from_parent_ptr = g_slist_next (traj_from_parent_ptr);
                }
                g_slist_free (node_curr->traj_from_parent);
                node_curr->traj_from_parent = NULL;
                // Free input_from_parent
                GSList *inputs_from_parent_ptr = node_curr->inputs_from_parent;
                while (inputs_from_parent_ptr) {
                    optsystem_free_input (self->optsys, (input_t *) (inputs_from_parent_ptr->data));
                    inputs_from_parent_ptr = g_slist_next (inputs_from_parent_ptr);
                }
                g_slist_free (node_curr->inputs_from_parent);
                node_curr->inputs_from_parent = NULL;
                
                // Free this state
                optsystem_free_state (self->optsys, state_curr);
                
            }
            else {   // If node_end is not given, then insert this state into the graph 
                node_curr = opttree_new_node_no_state ();
                node_curr->children = NULL;
                node_curr->state = state_curr;
                node_curr->reaches_target = optsystem_is_reaching_target (self->optsys, node_curr->state);
                
                kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);

                // Add node to the graph
                self->list_nodes = g_slist_prepend (self->list_nodes, (gpointer) (node_curr));
                self->num_nodes++;
                
            }
            node_curr->parent = node_prev;
            if (inputs) {
                input_t *input_this = (input_t *)(inputs_ptr->data);
                inputs_curr = g_slist_prepend (inputs_curr, input_this);
            }
            node_curr->inputs_from_parent = g_slist_reverse (inputs_curr);
            node_curr->traj_from_parent = g_slist_reverse (subtraj_curr); 
            node_curr->distance_from_parent = optsystem_evaluate_distance_for_cost (self->optsys,
                                                                                    node_curr->inputs_from_parent, obstacle_cost);
            opttree_update_distance_from_root (self, node_prev, node_curr);

            // Add this node to the children of the previous node
            node_prev->children = g_slist_prepend (node_prev->children, node_curr);

            // Reevaluate reaches_target variables
            if (node_curr->reaches_target) {
                if (! (node_prev->reaches_target) ) {
                    node_t *node_temp = node_prev;
                    while (node_temp )  {
                        node_temp->reaches_target = TRUE;
                        node_temp = node_temp->parent;
                    }
                }
                
                if (node_curr->reaches_target) {
                    if (node_curr->distance_from_root < self->lower_bound) {
                        self->lower_bound = node_curr->distance_from_root;
                        self->lower_bound_node = node_curr;
                    }
                }
            }
            
            // Reset the pointers
            subtraj_curr = NULL;
            node_prev = node_curr;
            
            goto end_iteration;
            
        }
        
        if (node_states) {
            if ( trajectory_count == node_states[node_states_count] ) {
                
                // Create the new node
                node_t *node_curr = opttree_new_node_no_state ();
                node_curr->state =state_curr;
                node_curr->parent = node_prev;
                node_curr->children = NULL;
                node_curr->reaches_target = optsystem_is_reaching_target (self->optsys, node_curr->state);
                if (inputs) {
                    inputs_curr = g_slist_prepend (inputs_curr, inputs_ptr->data);
                }
                node_curr->inputs_from_parent = g_slist_reverse (inputs_curr);
                node_curr->traj_from_parent = g_slist_reverse (subtraj_curr);
                node_curr->distance_from_parent = optsystem_evaluate_distance_for_cost (self->optsys,
                                                                                        node_curr->inputs_from_parent, obstacle_cost);
                node_curr->distance_from_root = node_prev->distance_from_root + node_curr->distance_from_parent;

                kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);

                // Update the parent node children pointer
                node_prev->children = g_slist_prepend (node_prev->children, node_curr);

                // Reevaluate reaches_target variables
                if (node_curr->reaches_target) {
                    if (! (node_prev->reaches_target) ) {
                        node_t *node_temp = node_prev;
                        while (node_temp )  {
                            node_temp->reaches_target = TRUE;
                            node_temp = node_temp->parent;
                        }
                    }

                    if (node_curr->distance_from_root < self->lower_bound) {
                        self->lower_bound = node_curr->distance_from_root;
                        self->lower_bound_node = node_curr;
                    }
                }

                self->list_nodes = g_slist_prepend (self->list_nodes, node_curr);
                self->num_nodes++;
                
                // Reset the pointers
                subtraj_curr = NULL;
                inputs_curr = NULL;

                node_prev = node_curr;

                // Increase the node_states counter
                node_states_count++;

                goto end_iteration;
                
            }
        }

        // Add current state to the subtrajectory
        subtraj_curr = g_slist_prepend (subtraj_curr, state_curr);
        if (inputs)
            inputs_curr = g_slist_prepend (inputs_curr, inputs_ptr->data);
        
    end_iteration:

        trajectory_ptr = g_slist_next (trajectory_ptr);
        if (inputs) {
            inputs_ptr = g_slist_next (inputs_ptr);
        }
        trajectory_count++;
    }
    
    g_slist_free (trajectory);
    g_slist_free (inputs);

    free (node_states);
            
    return node_prev;
}


// Extend node towards a given sample
// If node can not be extended , e.g., a collision occurs, then returns FALSE
node_t *opttree_extend_towards_sample (opttree_t *self, node_t *node_from, state_t *state_towards) {

    // Extend the node towards the sample
    int fully_extends = 0;
    GSList *trajectory = NULL;
    int num_node_states = 0;
    int  *node_states = NULL;
    GSList *inputs = NULL;
    double obstacle_cost = 0;
    gboolean stop_at_goal = 0;
    if (optsystem_extend_to (self->optsys, node_from->state, state_towards, 
                             &fully_extends, &trajectory, &num_node_states, &node_states, &inputs, &obstacle_cost, stop_at_goal) == 0) {
        return NULL;
    }
    
    // Add all the nodes in the trajectory to the tree
    node_t *node_curr = opttree_add_traj_to_graph (self, node_from, NULL, trajectory, num_node_states, node_states, inputs, obstacle_cost);
    
    // Check for reachability of the target, if so update the lower_bound
    if (optsystem_is_reaching_target (self->optsys, node_curr->state)) {
        if (node_curr->distance_from_root < self->lower_bound) {
            self->lower_bound = node_curr->distance_from_root;
            self->lower_bound_node = node_curr;
        }
        node_curr->reaches_target = 1;
    }
    else 
        node_curr->reaches_target = 0;
    
    return node_curr; // Return the last node
}


// Extends a given node towards state_towards and returns the resulting state
//    does not generate a new node or populate the tree
state_t *opttree_extend_towards_sample_no_create_node (opttree_t *self, node_t *node_from, state_t *state_towards) {

    state_t *state = NULL;

    int fully_extends = 0;
    GSList *trajectory = NULL;
    int num_node_states = 0;
    int *node_states = NULL;
    GSList *inputs = NULL;
    double obstacle_cost = 0;
    gboolean stop_at_goal = 1;
    if (optsystem_extend_to (self->optsys, node_from->state, state_towards, 
                             &fully_extends, &trajectory, &num_node_states, &node_states, &inputs, &obstacle_cost, stop_at_goal) == 0)
        return NULL;
   
    {
        // Get the last state in the trajectory
        GSList *state_trajectory = trajectory;
        while (state_trajectory) {
            state_t *state_curr = state_trajectory->data;
            if (!g_slist_next (state_trajectory)) 
                state = optsystem_clone_state (self->optsys, state_curr);
            state_trajectory = g_slist_next (state_trajectory); 
        }
    }
    
    {// Define some local variables
        GSList *state_trajectory = trajectory;
        while (state_trajectory) {
            state_t *state_curr = (state_t *)(state_trajectory->data);
            optsystem_free_state (self->optsys, state_curr);
            state_trajectory = g_slist_next (state_trajectory); 
        }
        g_slist_free (trajectory);
        GSList *inputs_ptr = inputs;
        while (inputs_ptr) {
            input_t *input_curr = (input_t *)(inputs_ptr->data);
            optsystem_free_input (self->optsys, input_curr);
            inputs_ptr = g_slist_next (inputs_ptr);
        }
        g_slist_free (inputs);
        
        // Free node_states
        free (node_states);
    }        
    return state;
}


// Goes through all the nodes in node_list and returns a pointer to the node that 
//    gets to state_towards with minimum cost
node_t* opttree_find_min_node_in_set (opttree_t *self, state_t *state_towards, GSList *list_nodes) {

    node_t *min_node = NULL;
    double min_cost = DBL_MAX;

    GSList *node_curr_list = list_nodes; 
    while (node_curr_list ){
        node_t *node_curr = (node_t *) node_curr_list->data;
        
        
        // Extend this node towards state_towards and evaluate the cost incurred
        //    -- if the node does not extend, then move on             
        int fully_extends = 0;
        GSList *trajectory = NULL;
        int num_node_states = 0;
        int *node_states = NULL;
        GSList *inputs = NULL;
        double obstacle_cost = 0;
        gboolean stop_at_goal = 0;

        if (optsystem_extend_to (self->optsys, node_curr->state, state_towards, 
                                 &fully_extends, &trajectory, &num_node_states, &node_states, &inputs, &obstacle_cost,
                                 stop_at_goal) != 0) {
            
            
            if (fully_extends)
                {
                    // Evaluate the total cost
                    double total_cost = node_curr->distance_from_root;
                    
                    double incremental_cost = optsystem_evaluate_distance_for_cost (self->optsys, inputs, obstacle_cost);
                    
                    total_cost += incremental_cost;
                    
                    if (total_cost < min_cost) {
                        min_node = node_curr;
                        min_cost = total_cost;
                    }
                }
            
            // Free the states
            {
                GSList* traj_temp = trajectory;
                while (traj_temp) {
                    state_t *state_this = (state_t *)(traj_temp->data);
                    optsystem_free_state (self->optsys, state_this);
                    traj_temp = g_slist_next (traj_temp);
                }
            }
            // Free the trajectory 
            g_slist_free (trajectory); 
            // Free Inputs
            {
                GSList *inputs_temp = inputs; 
                while (inputs_temp) {
                    input_t *input_this = (input_t *)(inputs_temp->data);
                    optsystem_free_input (self->optsys, input_this);
                    inputs_temp = g_slist_next (inputs_temp);
                }
            }
            // Free the inputs list
            g_slist_free (inputs);
            // Free node_states
            free (node_states);
            
        }
        node_curr_list = g_slist_next (node_curr_list);
    }
    

    return min_node;}


// Takes a kdres set and returns a list of nodes
GSList *opttree_kdtree_to_gslist (state_t * state, kdres_t *kdres) {
    
    GSList *node_list = NULL;

    kd_res_rewind (kdres);
    while (!kd_res_end(kdres)) {
        node_t * node_curr = kd_res_item_data (kdres);
        node_list = g_slist_prepend (node_list, node_curr);
        kd_res_next (kdres);
    }
    
    return node_list;
}


// Finds the set of nodes with max cost
GSList *opttree_find_nodes_in_ball (opttree_t *self, state_t *state, double ball_radius) {

    GSList *nodes_in_ball = NULL; 

    kdres_t *kdres = kd_nearest_range (self->kdtree, optsystem_get_state_key (self->optsys, state), ball_radius);
    nodes_in_ball = opttree_kdtree_to_gslist (state, kdres);
    kd_res_free (kdres);
    
    return nodes_in_ball;
}


// Extends the node back to the tree 
int opttree_extend_back_to_tree (opttree_t *self, node_t *node_from, GSList *node_list) {
    

    state_t *state_from = node_from->state;

    GSList *node_curr_list = node_list;    
    while (node_curr_list) {
        node_t *node_curr = (node_t *) (node_curr_list->data);
        
        if (node_curr == node_from){ // If current node is the same node_from, then continue normal operation
            node_curr_list = g_slist_next (node_curr_list);
            continue;
        }

        if (node_curr == self->root) {
            node_curr_list = g_slist_next (node_curr_list);
            continue;
        }

        state_t *state_towards = node_curr->state;


        gboolean free_states = FALSE;

        // Try to extend the state towards the sample
        int fully_extends = 0; 
        GSList *trajectory = NULL;
        int num_node_states = 0;
        int *node_states = NULL;
        GSList *inputs = NULL;
        double obstacle_cost = 0;
        gboolean stop_at_goal = 0;
        if (optsystem_extend_to (self->optsys, state_from, state_towards, 
                                 &fully_extends, &trajectory, &num_node_states, &node_states, &inputs, &obstacle_cost,
                                 stop_at_goal) == 0) {
            fully_extends = 0;
        }
        
        if (fully_extends) {   // Consider rewiring the tree
            
            // Calculate the cummulative distance from the root till the end of the trajectory
            double dist = node_from->distance_from_root;
            dist += optsystem_evaluate_distance_for_cost (self->optsys, inputs, obstacle_cost);

            // Check whether the new branch is shorter than the existing one 
            if (dist < node_curr->distance_from_root) {

                // Add the trajectory to the tree
                node_t *node_new; 
                node_new = opttree_add_traj_to_graph (self, node_from, node_curr, trajectory, num_node_states, node_states, inputs, obstacle_cost);
            } 
            else {                   // If the new distance from the root is not less than the current one
                free_states = TRUE;  // remember to free up the memory used by the states in the trajectory
            }
        } 
        else{                      // If it does not fully extend
            free_states = TRUE;    // remember to free up the memory used by the states in the trajectory
        }                          //     OBSTACLES CAUSE NO EXTENSIONS, IN WHICH CASE fully_extend = 0
        
        // Free up the memory used by the trajectory if the trajectory is not registered into the graph
        if (free_states) {
            GSList *trajectory_ptr = trajectory;
            while (trajectory_ptr) {
                optsystem_free_state (self->optsys, (state_t *)(trajectory_ptr->data));
                trajectory_ptr = g_slist_next (trajectory_ptr);
            }
            g_slist_free (trajectory);
            GSList *inputs_ptr = inputs;
            while (inputs_ptr) {
                optsystem_free_input (self->optsys, (input_t *)(inputs_ptr->data));
                inputs_ptr = g_slist_next (inputs_ptr);
            }
            g_slist_free (inputs);
            free (node_states);
        }

        node_curr_list = g_slist_next (node_curr_list);
    } 
    return 1;
}


int opttree_iteration (opttree_t *self) {

    // 1. Sample a state
    state_t state_random;
    
    //    Target sampling rule
    if (self->lower_bound_node) {
        if (rand()/(RAND_MAX + 1.0) > self->target_sample_prob_after_first_solution) {
            if (optsystem_sample_state (self->optsys, &state_random) == 0)
                return 0;
        }
        else {
            if (optsystem_sample_target_state (self->optsys, &state_random) == 0)
                return 0;
        }
    }
    else 
        {
            if (rand()/(RAND_MAX + 1.0) > self->target_sample_prob_before_first_solution) {
                if (optsystem_sample_state (self->optsys, &state_random) == 0)
                    return 0;
            }
            else {
                if (optsystem_sample_target_state (self->optsys, &state_random) == 0)
                    return 0;
            }
        }
        
    // A) RRT* ALGORITHM
    if (self->run_rrtstar) {   
        
        // A.1. Calculate the ball radius constant
        self->ball_radius_last = self->ball_radius_constant 
            * (pow(log(1+(double)(self->num_nodes))/((double)(self->num_nodes)), 1.0/NUM_STATES));
        if (self->ball_radius_last >= self->ball_radius_max)
            self->ball_radius_last = self->ball_radius_max;

        // A.2. Find the nearest node
        node_t *nearest_node = opttree_find_nearest_neighbor (self, &state_random);
        if (!nearest_node)
            return 0;
        
        // A.3. Extend the node towards the sampled state -- just computer the extended state
        state_t *extended_state = opttree_extend_towards_sample_no_create_node (self, nearest_node, &state_random);
        if (!extended_state)
            return 0;
        
        // A.4. Compute the set of all close nodes around extended_state
        GSList *close_nodes = NULL;
        close_nodes = opttree_find_nodes_in_ball (self, extended_state, self->ball_radius_last);        

        // A.5. Pick the node to be extended
        node_t *node_from = nearest_node;  // If close_nodes is empty,     then extend nearest_nodes
        if (close_nodes) {                 // If close_nodes is non-empty, then extend the min_node in close_nodes
            node_from = opttree_find_min_node_in_set (self, extended_state, close_nodes);
            if (!node_from)                //   If no close node can be extended, then fall back to the nearest 
                node_from = nearest_node;
        }
        
        // A.6. Extend the appropriate node
        node_t *extended_node = opttree_extend_towards_sample (self, node_from, extended_state);
        if (!extended_node) {
            g_slist_free (close_nodes);
            return 0;
        }

        // A.7. Rewire the tree if possible
        opttree_extend_back_to_tree (self, extended_node, close_nodes);
        
        optsystem_free_state (self->optsys, extended_state);
        
        g_slist_free (close_nodes);
    }
    
    // B) RRT ALGORITHM
    else {
        
        // B.1. Find the nearest node
        node_t *nearest_node = opttree_find_nearest_neighbor (self, &state_random);
        if (!nearest_node)
            return 0;
        
        // B.2. Extend the nearest towards the sample
        node_t *extended_node = opttree_extend_towards_sample (self, nearest_node, &state_random);
        if (!extended_node)
            return 0;
    }

    return 1;
} 

int opttree_reset (opttree_t *self) {

    // Clear the tree
    GSList *node_curr_list  = self->list_nodes; 
    while (node_curr_list) {
        node_t *node_curr = node_curr_list->data;
        
        node_curr->reaches_target = FALSE;

        node_curr_list = g_slist_next (node_curr_list); 
    }

    // Reinitialize the basics
    self->lower_bound = DBL_MAX;

    //save the current goal
    self->lower_bound_node_to_first_goal = self->lower_bound_node; 
    self->lower_bound_node = NULL;

    return 1;
}


int opttree_reinitialize (opttree_t *self) {

    // Clear the tree
    GSList *node_curr_list  = self->list_nodes; 
    while (node_curr_list) {
        node_t *node_curr = node_curr_list->data;
        
        opttree_free_node (self, node_curr);

        node_curr_list = g_slist_next (node_curr_list); 
    }

    g_slist_free (self->list_nodes);
    
    self->list_nodes = NULL;

    // Reinitialize the basics
    self->lower_bound = DBL_MAX;
    self->lower_bound_node = NULL;
    self->lower_bound_node_to_first_goal = NULL;

    // Reinitialize the kdtree
    kd_clear (self->kdtree);

    // Initialize the root node
    self->root = opttree_new_node (self);
    optsystem_get_initial_state (self->optsys, self->root->state);
    self->root->distance_from_root = 0.0;
    self->root->distance_from_parent = 0.0;
    kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, self->root->state), self->root);

    // Initialize the list of all nodes
    self->list_nodes = NULL;
    self->list_nodes = g_slist_prepend (self->list_nodes, (gpointer) (self->root));
    self->num_nodes = 1;

    return 1;
}


opttree_t* opttree_create (gboolean sensing_only_local, gboolean draw, gboolean clear_using_laser, gboolean sensing_only_small) { 

    opttree_t *self = (opttree_t *) calloc (sizeof (opttree_t), 1);

    // Set up the dynamical system
    self->optsys = (optsystem_t *) calloc (sizeof (optsystem_t), 1);
    optsystem_new_system (self->optsys, sensing_only_local, draw, clear_using_laser, sensing_only_small);
    
    // Initialize the kdtree
    self->kdtree = kd_create (optsystem_get_num_states(self->optsys));

    // Set the lower bound to infinity
    self->lower_bound = DBL_MAX;
    self->lower_bound_node = NULL;
    self->lower_bound_node_to_first_goal = NULL;

    // Initialize the root node
    self->root = opttree_new_node (self);
    optsystem_get_initial_state (self->optsys, self->root->state);
    self->root->distance_from_root = 0.0;
    self->root->distance_from_parent = 0.0;

    kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, self->root->state), self->root);

    // Initialize the list of all nodes
    self->list_nodes = NULL;
    self->list_nodes = g_slist_prepend (self->list_nodes, (gpointer) (self->root));
    self->num_nodes = 1;

    // Initialize parameters to default values
    self->run_rrtstar = 1;
    self->ball_radius_constant = 30.0;
    self->ball_radius_max = 1.0;
    self->target_sample_prob_after_first_solution = 0.0;
    self->target_sample_prob_before_first_solution = 0.0;
    return self;
}


// Frees the memory allocated for the nodes of the tree 
int opttree_free_tree (opttree_t *self) {

    GSList *list_node_curr = self->list_nodes;
    while (list_node_curr) {
        opttree_free_node (self, (node_t *)(list_node_curr->data) );
        list_node_curr = g_slist_next (list_node_curr);
    }
    g_slist_free (self->list_nodes);
    
    return 1;
}


int opttree_destroy (opttree_t *self) {

    optsystem_free_system (self->optsys);

    opttree_free_tree (self);

    return 1;
}


int opttree_set_root_state (opttree_t *self, state_t *state) {

    optsystem_set_initial_state (self->optsys, state); 
    optsystem_get_initial_state (self->optsys, self->root->state);

    return 1;
} 


// == Branch and bound

int opttree_remove_branch (opttree_t *self, node_t *node_curr) {

    // Create a temporary children list
    GSList *node_child_list_tmp = NULL;
    GSList *node_child_list = node_curr->children;
    while (node_child_list) {
        node_t *node_child = node_child_list->data;
        node_child_list_tmp = g_slist_prepend (node_child_list_tmp, node_child);
        node_child_list = g_slist_next (node_child_list);
    }

    // Recursive call to children
    while (node_child_list_tmp) {
        node_t *node_child = node_child_list_tmp->data;
        opttree_remove_branch (self, node_child);
        node_child_list_tmp = g_slist_next (node_child_list_tmp);
    }
    g_slist_free (node_child_list_tmp);
    
    if (node_curr->children)
        printf ("ERROR: How come there is any children\n");

    // Delete the node from the parent's children list
    node_t *node_parent  = node_curr->parent;
    if (node_parent) 
        node_parent->children = g_slist_remove (node_parent->children, node_curr);
    else 
        printf ("Node doesn't have parent\n");
    self->list_nodes = g_slist_remove (self->list_nodes, node_curr);

    opttree_free_node (self, node_curr);
        
    return 1;
}


int opttree_branch_and_bound (opttree_t *self) {    
    
    int node_deleted;
    do {
        node_deleted = 0;

        GSList *node_curr_list  = self->list_nodes; 

        // Form the list for the best trajectory
        GSList *traj_best = NULL;
        node_t *best_node_curr = self->lower_bound_node;
        while (best_node_curr) {
            traj_best = g_slist_prepend (traj_best, best_node_curr);
            best_node_curr = best_node_curr->parent;
        }

        while (node_curr_list) {
            node_t *node_curr = node_curr_list->data;

            // Don't remove this node if it's along the current solution path to the root
            int keep_node = opttree_check_if_solution_in_decendants (self, traj_best, node_curr);

            // Estimate of the best-case cost through the node to the goal
            double admissible_cost_estimate = node_curr->distance_from_root 
                + optsystem_evaluate_cost_to_go (self->optsys, node_curr->state);

            if (!keep_node && (admissible_cost_estimate > self->lower_bound + BNB_TOLERANCE) ) {
                opttree_remove_branch (self, node_curr);
                node_deleted = 1;
                break;

            }
            node_curr_list = g_slist_next (node_curr_list); 
        }

    } while (node_deleted);

    // Destroy the kd-tree and rebuild
    int64_t time_start_kd = bot_timestamp_now ();
    kd_clear (self->kdtree);
    

    self->lower_bound = DBL_MAX;
    self->lower_bound_node = NULL;

    //do we trash this ??
    //self->lower_bound_node_to_first_goal = NULL;

    self->num_nodes = 0;

    GSList *nodes_ptr = self->list_nodes;
    if (!nodes_ptr)
        fprintf (stderr, "ERROR: nodes_ptr is NULL. Branch and bound removed entire tree!\n");
    while (nodes_ptr) {
        node_t *node_curr = nodes_ptr->data;
        if ( optsystem_is_reaching_target(self->optsys, node_curr->state) && (node_curr->distance_from_root < self->lower_bound)) {
            self->lower_bound = node_curr->distance_from_root;
            self->lower_bound_node = node_curr;
        }
        kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);
        self->num_nodes++;
        nodes_ptr = g_slist_next (nodes_ptr);
    }
    
    //printf ("B&B complete!   Time: %2.6lf\n", ((double)(bot_timestamp_now () - time_start_kd))/1000000.0 );

    return 1;
}


int
opttree_check_if_solution_in_decendants (opttree_t *self, GSList *node_list, node_t *node_check)
{
    if (!node_list)
        return 0;

    
    while (node_list) {
        // Hack to check whether the node_check and node_curr are the same
        node_t *node_curr = (node_t *)node_list->data;// self->lower_bound_node;
        if (node_curr->distance_from_root == node_check->distance_from_root)
            return 1;
        node_list = g_slist_next (node_list);
    }

    return 0;
}

// == Committed traj

GSList* opttree_determine_decendants (opttree_t *self, GSList *node_list, node_t *node_curr){

    GSList *child_node_ptr = node_curr->children;
    while (child_node_ptr) {
        node_t *child_node = (node_t *)(child_node_ptr->data);
        node_list = opttree_determine_decendants (self, node_list, child_node);
        child_node_ptr = g_slist_next (child_node_ptr);
    }
    node_list = g_slist_prepend (node_list, node_curr);
    return node_list;
}


GSList* opttree_determine_traj_decendants (opttree_t *self, GSList *node_list, node_t *node_curr, node_t *node_next) {

    if (!node_curr)
        return NULL;
    GSList *child_node_ptr = node_curr->children;
    while (child_node_ptr) {
        node_t *child_node = (node_t *)(child_node_ptr->data);
        if (child_node != node_next) 
            node_list = opttree_determine_decendants (self, node_list, child_node);
        child_node_ptr = g_slist_next (child_node_ptr);
    }
    node_list = g_slist_prepend (node_list, node_curr);
    if (node_curr->parent) 
        node_list = opttree_determine_traj_decendants (self, node_list, node_curr->parent, node_curr);
    return node_list;
}


int opttree_reevaluate_distance_from_parent (opttree_t *self, node_t *node_curr) {
    
    GSList *children_ptr = node_curr->children;
    while (children_ptr) {
        node_t *node_child = (node_t *)(children_ptr->data);
        node_child->distance_from_root = node_curr->distance_from_root + node_child->distance_from_parent;
        opttree_reevaluate_distance_from_parent (self, node_child);
        children_ptr = g_slist_next (children_ptr);
    }
    
    return 1;
}


// TODO: THIS CODE IS VERY BADLY STRUCTURED, REDO THIS...

int opttree_eliminate_nodes (opttree_t *self, GSList *node_list, node_t *root_new,
                             node_t *root_node_new, GSList *traj_from_parent_new, GSList *inputs_from_parent_new) {

  

    // Delete all decendant nodes
    //printf ("num_list_nodes_before : %d\n", g_slist_length (self->list_nodes));
    GSList *node_list_ptr = node_list;
    while (node_list_ptr) {
        node_t *node_curr = (node_t *)(node_list_ptr->data);
        self->list_nodes = g_slist_remove (self->list_nodes, node_curr);
        opttree_free_node (self, node_curr);
        node_list_ptr = g_slist_next (node_list_ptr);
    }
    //     printf ("num_list_nodes_after  : %d\n", g_slist_length (self->list_nodes));

    // Delete the root node and add it to the end of the list
    self->list_nodes = g_slist_remove (self->list_nodes, root_new);
    self->list_nodes = g_slist_append (self->list_nodes, root_new);
    
    // Change the root node to node_commit and declare its parent null
    root_new->parent = NULL;
    root_new->distance_from_parent = 0.0;
    root_new->distance_from_root = 0.0;
    GSList *input_ptr = root_new->inputs_from_parent;
    while (input_ptr) {
        input_t *input_curr = (input_t *)(input_ptr->data);
        optsystem_free_input (self->optsys, input_curr);
        input_ptr = g_slist_next (input_ptr);
    }
    g_slist_free (root_new->inputs_from_parent);
    root_new->inputs_from_parent = NULL;
    GSList *traj_ptr = root_new->traj_from_parent;
    while (traj_ptr) {
        state_t *state_curr = (state_t *)(traj_ptr->data);
        optsystem_free_state (self->optsys, state_curr);
        traj_ptr = g_slist_next (traj_ptr);
    }
    g_slist_free (root_new->traj_from_parent);
    root_new->traj_from_parent = NULL;
    self->root = root_new;
    optsystem_set_initial_state (self->optsys, self->root->state); 


    // Modify the root node, if necessary
    if (root_node_new) {
        //printf ("list_lengths : %d - %d\n", g_slist_length (traj_from_parent_new), g_slist_length (inputs_from_parent_new));
        GSList *state_ptr = traj_from_parent_new;
        while (state_ptr) {
            state_t *state_curr = (state_t *)(state_ptr->data);
            //fprintf (stdout, "(%3.5lf, %3.5lf)\n", state_curr->x[0], state_curr->x[1]);
            state_ptr = g_slist_next (state_ptr);
        }
        GSList *input_ptr = inputs_from_parent_new;
        while (input_ptr) {
            input_t *input_curr = (input_t *)(input_ptr->data);
            //fprintf (stdout, "(%3.5lf, %3.5lf)\n", input_curr->x[0], input_curr->x[1]);
            input_ptr = g_slist_next (input_ptr);
        }
        root_node_new->children = g_slist_prepend (root_node_new->children, root_new);
        root_new->parent = root_node_new; 
        root_new->traj_from_parent = traj_from_parent_new;
        root_new->inputs_from_parent = inputs_from_parent_new;
        root_new->distance_from_parent = optsystem_evaluate_distance_for_cost (self->optsys, inputs_from_parent_new, 0.0);
        
        self->list_nodes = g_slist_append (self->list_nodes, root_node_new);
        self->num_nodes++;

        self->root = root_node_new;
        optsystem_set_initial_state (self->optsys, self->root->state); 
    }

    opttree_reevaluate_distance_from_parent (self, self->root);


    // Setup the kd-tree
    kd_clear (self->kdtree);
    self->lower_bound = DBL_MAX;
    self->lower_bound_node = NULL;
    self->num_nodes = 0;
    GSList *nodes_ptr = self->list_nodes;
    while (nodes_ptr) {
        node_t *node_curr = nodes_ptr->data;
        if ( optsystem_is_reaching_target(self->optsys, node_curr->state) && (node_curr->distance_from_root < self->lower_bound)) {
            self->lower_bound = node_curr->distance_from_root;
            self->lower_bound_node = node_curr;
        }
        kd_insert (self->kdtree, optsystem_get_state_key (self->optsys, node_curr->state), node_curr);
        self->num_nodes++;
        nodes_ptr = g_slist_next (nodes_ptr);
    }
    
    return 1;
}


int opttree_commit_traj_with_node (opttree_t *self, node_t *node_commit, 
                                   node_t *root_node_new, GSList *traj_from_parent_new, GSList *inputs_from_parent_new) {

    // Delete the committed part of the trajectory and its decendants from the tree and update the kd-tree
    GSList *nodes_commit_traj_decendants = NULL;
    nodes_commit_traj_decendants = opttree_determine_traj_decendants (self, nodes_commit_traj_decendants, 
                                                                      node_commit->parent, node_commit);

    // Eliminate the nodes and restructure the tree
    opttree_eliminate_nodes (self, nodes_commit_traj_decendants, node_commit, 
                             root_node_new, traj_from_parent_new, inputs_from_parent_new);
    
    return 1;
}

GSList* opttree_commit_traj_all (opttree_t *self) { 
    //commit all the rest of the trajectory 

    if (!self->lower_bound_node)
        return NULL;

    // get the best trajectory
    GSList *traj_best = NULL;
    node_t *node_curr = self->lower_bound_node;
    while (node_curr) {
        traj_best = g_slist_prepend (traj_best, node_curr);
        node_curr = node_curr->parent;
    }
    
    // find the first node that is ahead of 'time' length
    GSList *traj_ptr = traj_best;
    double time_total = 0.0;
    node_t *traj_node_curr = NULL;
    input_t *input_cut = NULL;
    while (traj_ptr) {

        traj_node_curr = (node_t *)(traj_ptr->data);
        
        // Find the time to get to this node
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
        while (input_ptr) {
            input_t *input_curr = (input_t *)(input_ptr->data);
            time_total += input_curr->x[1];
            //printf ("time_total : %3.5lf \n", time_total);
            input_ptr = g_slist_next (input_ptr);
        }
        //printf ("TIME_TOTAL : %3.5lf \n", time_total);

        traj_ptr = g_slist_next (traj_ptr);
    }

    if (traj_node_curr == NULL) {
        g_slist_free (traj_best);
        return NULL;
    }
    
    // Determine the root trajectory
    node_t *root_node_new = NULL;
    GSList *traj_from_parent_new = NULL;
    GSList *inputs_from_parent_new = NULL;
    state_t *root_state = NULL;
    if (input_cut) {
        GSList *state_ptr = traj_node_curr->traj_from_parent;
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
        while (input_ptr) {
            state_t *state_curr = NULL;
            if(state_ptr) 
                state_curr = (state_t *)(state_ptr->data);
            input_t *input_curr = (input_t *)(input_ptr->data);
            if (input_curr == input_cut) {
                if (!state_ptr) { 
                    //this is fine here as we are committing everything 
                    root_state = NULL;
                }
                else {
                    root_state = state_curr;
                    while (input_ptr) {
                        input_curr = (input_t *)(input_ptr->data);
                        inputs_from_parent_new = g_slist_prepend (inputs_from_parent_new, 
                                                                  optsystem_clone_input (self->optsys, input_curr));
                        input_ptr = g_slist_next (input_ptr);
                        if(state_ptr) {
                            state_curr = (state_t *)(state_ptr->data);
                            traj_from_parent_new = g_slist_prepend (traj_from_parent_new, 
                                                                    optsystem_clone_state (self->optsys, state_curr) );
                            state_ptr = g_slist_next (state_ptr);
                        }
                    }
                }
            }
            if (state_ptr)
                state_ptr = g_slist_next (state_ptr);
            input_ptr = g_slist_next (input_ptr);
        }
        
        if (root_state) {
            root_node_new = opttree_new_node (self);
            root_node_new->distance_from_root = 0.0;
            root_node_new->distance_from_parent = 0.0;
            root_node_new->state = optsystem_clone_state (self->optsys, root_state);
            inputs_from_parent_new = g_slist_reverse (inputs_from_parent_new);
            traj_from_parent_new = g_slist_reverse (traj_from_parent_new);
        }
    }

    // Generate the committed trajectory
    GSList *committed_traj = NULL;
    GSList *traj_best_ptr = traj_best; 
    while (traj_best_ptr) {
        gboolean loop_break = FALSE;

        node_t *node_this = (node_t *)(traj_best_ptr->data);
        
        // Add traj_from_parent of node_this
        GSList *state_ptr = node_this->traj_from_parent;
        while (state_ptr) {
            state_t *state_this = (state_t *)(state_ptr->data);
            committed_traj = g_slist_prepend (committed_traj, 
                                             optsystem_clone_state(self->optsys, state_this) );
            if (state_this == root_state) {
                loop_break = TRUE;
                break;
            }
            state_ptr = g_slist_next (state_ptr);
        }
        if(loop_break)
            break;
        committed_traj = g_slist_prepend (committed_traj, 
                                         optsystem_clone_state (self->optsys, node_this->state));
        traj_best_ptr = g_slist_next (traj_best_ptr);
    }
    committed_traj = g_slist_reverse (committed_traj);
    
    // Commit to this node 
    opttree_commit_traj_with_node (self, traj_node_curr, root_node_new, traj_from_parent_new, inputs_from_parent_new);


    //printf ("commit traj end\n");
    
    return committed_traj;
}

int opttree_get_commit_end_point_to_old_goal (opttree_t *self, double time, 
                                   double *new_root_node) {

    int all_committed = 0;

    if (!self->lower_bound_node_to_first_goal)
        return -1;

    // get the best trajectory
    GSList *traj_best = NULL;
    node_t *node_curr = self->lower_bound_node_to_first_goal;
    while (node_curr) {
        traj_best = g_slist_prepend (traj_best, node_curr);
        node_curr = node_curr->parent;
    }    
    

    // find the first node that is ahead of 'time' length
    GSList *traj_ptr = traj_best;
    double time_total = 0.0;
    node_t *traj_node_curr = NULL;
    input_t *input_cut = NULL;

    traj_node_curr = (node_t *)(traj_ptr->data);

    while (traj_ptr) {

        traj_node_curr = (node_t *)(traj_ptr->data);    
        
        // Find the time to get to this node
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
        while (input_ptr) {
            input_t *input_curr = (input_t *)(input_ptr->data);
            time_total += input_curr->x[1];
            //printf ("time_total : %3.5lf \n", time_total);
            if (time_total > time) {
                input_cut = input_curr;
                break;
            }
            input_ptr = g_slist_next (input_ptr);
        }
        if (time_total >= time)
            break;
        //printf ("TIME_TOTAL : %3.5lf \n", time_total);

        traj_ptr = g_slist_next (traj_ptr);
    }

    if (time_total < time) {
        all_committed = 1;
        //fprintf(stderr,"Remaining Path Committed\n");
    }
    if (traj_node_curr == NULL) {
        g_slist_free (traj_best);
        return -1;
    }
    
    // Determine the root trajectory
    node_t *root_node_new = NULL;
    GSList *traj_from_parent_new = NULL;
    GSList *inputs_from_parent_new = NULL;
    state_t *root_state = NULL;
    if (input_cut) {
        GSList *state_ptr = traj_node_curr->traj_from_parent;
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
        while (input_ptr) {
            state_t *state_curr = NULL;
            if(state_ptr) 
                state_curr = (state_t *)(state_ptr->data);
            input_t *input_curr = (input_t *)(input_ptr->data);
            if (input_curr == input_cut) {
                if (!state_ptr && !(all_committed)){
                    root_state = traj_node_curr->state;//NULL;
                }
                else{
                    root_state = state_curr;                    
                }
            }
            if (state_ptr)
                state_ptr = g_slist_next (state_ptr);
            input_ptr = g_slist_next (input_ptr);
        }
        
        if (root_state) {
            //fprintf(stderr,"************** New Root Node : %f,%f,%f\n", 
            //        root_state->x[0], root_state->x[1], root_state->x[2]); 
            new_root_node[0] = root_state->x[0];
            new_root_node[1] = root_state->x[1];
            new_root_node[2] = root_state->x[2];

            return 0;            
        }
        else if(!(all_committed)){
            fprintf(stderr, "ERROR: New Root node is NULL\n");
            return 1;
        }
        else{
            return 1; 
        }
    }
    if(!(all_committed)){
        fprintf(stderr, "ERROR: New Root node is NULL\n");
        return 1; 
    }
    else{
        return 1;
    }

}


int opttree_get_commit_end_point (opttree_t *self, double time, 
                                   double *new_root_node) {

    //printf ("commit traj start\n");

    int all_committed = 0;

    if (!self->lower_bound_node)
        return -1;

    // get the best trajectory
    GSList *traj_best = NULL;
    node_t *node_curr = self->lower_bound_node;
    while (node_curr) {
        traj_best = g_slist_prepend (traj_best, node_curr);
        node_curr = node_curr->parent;
    }

    // find the first node that is ahead of 'time' length
    GSList *traj_ptr = traj_best;
    double time_total = 0.0;
    node_t *traj_node_curr = NULL;
    input_t *input_cut = NULL;
    
    traj_node_curr = (node_t *)(traj_ptr->data);

    GSList *input_cut_ptr; 

    //we do find the root node here 

    while (traj_ptr) {

        traj_node_curr = (node_t *)(traj_ptr->data);
        
        // Find the time to get to this node
        GSList *input_ptr = traj_node_curr->inputs_from_parent;

        while (input_ptr) {
            input_t *input_curr = (input_t *)(input_ptr->data);
            time_total += input_curr->x[1];
            //printf ("time_total : %3.5lf \n", time_total);
            if (time_total > time) {
                //fprintf(stderr, "++++++  Input Cut \n");
                input_cut = input_curr;
                input_cut_ptr = input_ptr; 
                break;
            }
            input_ptr = g_slist_next (input_ptr);
        }
        if (time_total >= time)
            break;
        //printf ("TIME_TOTAL : %3.5lf \n", time_total);

        traj_ptr = g_slist_next (traj_ptr);
    }

    if (time_total < time) {
        all_committed = 1;
    }

    if (traj_node_curr == NULL) {
        g_slist_free (traj_best);
        return -1;
    }
    
    // Determine the root trajectory
    node_t *root_node_new = NULL;
    GSList *traj_from_parent_new = NULL;
    GSList *inputs_from_parent_new = NULL;
    state_t *root_state = NULL;
    if (input_cut) {
      
        int input_count = 0;
        int state_count = 0;
      
        GSList *state_ptr = traj_node_curr->traj_from_parent;
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
      
        while (input_ptr) {
            state_t *state_curr = NULL;

            if(state_ptr) 
                state_curr = (state_t *)(state_ptr->data);
            input_t *input_curr = (input_t *)(input_ptr->data);
            if (input_curr == input_cut) { //create the new root node 
                if (!state_ptr && !(all_committed)) { //commit until current state
                    root_state = traj_node_curr->state;
                }
                else {
                    root_state = state_curr;
                }
            }
            if (state_ptr){
                state_ptr = g_slist_next (state_ptr);
                state_count++;
            }
            
            input_ptr = g_slist_next (input_ptr);
            if(input_ptr){
                input_count++;
            }

            /* if(input_ptr && !state_ptr){ */
            /*     fprintf(stderr, "Point is the current State\n"); */
            /* } */
        }

        //fprintf(stdout, "Input Count : %d State Count : %d\n", input_count, state_count);
        
        if (root_state) {
            //fprintf(stderr,"************** New Root Node : %f,%f,%f\n", 
            //        root_state->x[0], root_state->x[1], root_state->x[2]); 
            new_root_node[0] = root_state->x[0];
            new_root_node[1] = root_state->x[1];
            new_root_node[2] = root_state->x[2];

            return 0;
            
            /*root_node_new = opttree_new_node (self);
            root_node_new->distance_from_root = 0.0;
            root_node_new->distance_from_parent = 0.0;
            root_node_new->state = optsystem_clone_state (self->optsys, root_state);
            inputs_from_parent_new = g_slist_reverse (inputs_from_parent_new);
            traj_from_parent_new = g_slist_reverse (traj_from_parent_new);*/
        }

        else if(!(all_committed)){
            fprintf(stderr, "ERROR: New Root node is NULL\n");
            return 1; 
        }
        else{
            return 1;
        }
    }

    if(!(all_committed)){
        fprintf(stderr, "ERROR: New Root node is NULL\n");
        return 1; 
    }
    else{
        return 1;
    }
}


GSList* opttree_commit_traj (opttree_t *self, double time, int *all_committed) {

    if (!self->lower_bound_node) {
        fprintf (stderr, "ERROR: opttree_commit_traj: lower_bound_node = NULL\n");
        return NULL;
    }

    // get the best trajectory
    GSList *traj_best = NULL;
    node_t *node_curr = self->lower_bound_node;
    while (node_curr) {
        traj_best = g_slist_prepend (traj_best, node_curr);
        node_curr = node_curr->parent;
    }

    // find the first node that is ahead of 'time' length
    GSList *traj_ptr = traj_best;
    double time_total = 0.0;
    node_t *traj_node_curr = NULL;
    input_t *input_cut = NULL;
    
    traj_node_curr = (node_t *)(traj_ptr->data);

    GSList *input_cut_ptr; 

    if (!traj_ptr)
         fprintf (stderr, "ERROR: opttree_commit_traj: traj_ptr = NULL\n");

    //we do find the root node here 
    while (traj_ptr) {

        traj_node_curr = (node_t *)(traj_ptr->data);
        
        // Find the time to get to this node
        GSList *input_ptr = traj_node_curr->inputs_from_parent;

        while (input_ptr) {
            input_t *input_curr = (input_t *)(input_ptr->data);
            time_total += input_curr->x[1];
            //printf ("time_total : %3.5lf \n", time_total);
            if (time_total > time) {
                //fprintf(stderr, "++++++  Input Cut \n");
                input_cut = input_curr;
                input_cut_ptr = input_ptr; 
                break;
            }
            input_ptr = g_slist_next (input_ptr);
        }
        if (time_total >= time) {
            break;
        }

        traj_ptr = g_slist_next (traj_ptr);
    }

    if (time_total < time) {
        *all_committed = 1;
    }
    if (traj_node_curr == NULL) {
        g_slist_free (traj_best);
        fprintf (stderr, "ERROR: opttree_commit_traj: traj_best = NULL\n");
        return NULL;
    }
    
    // Determine the root trajectory
    node_t *root_node_new = NULL;
    GSList *traj_from_parent_new = NULL;
    GSList *inputs_from_parent_new = NULL;
    state_t *root_state = NULL;
    if (input_cut) {
      
        int input_count = 0;
        int state_count = 0;
      
        GSList *state_ptr = traj_node_curr->traj_from_parent;
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
      
        while (input_ptr) {
            state_t *state_curr = NULL;

            if(state_ptr) 
                state_curr = (state_t *)(state_ptr->data);
            input_t *input_curr = (input_t *)(input_ptr->data);
            if (input_curr == input_cut) { //create the new root node 
                if (!state_ptr && !(*all_committed)) { //commit until current state
                    root_state = traj_node_curr->state;
                }
                else {
                    root_state = state_curr;
                    while (input_ptr) {
                        input_curr = (input_t *)(input_ptr->data);
                        inputs_from_parent_new = g_slist_prepend (inputs_from_parent_new, 
                                                                  optsystem_clone_input (self->optsys, input_curr));
                        input_ptr = g_slist_next (input_ptr);
                        if(state_ptr) {
                            state_curr = (state_t *)(state_ptr->data);
                            traj_from_parent_new = g_slist_prepend (traj_from_parent_new, 
                                                                    optsystem_clone_state (self->optsys, state_curr) );
                            state_ptr = g_slist_next (state_ptr);
                        }
                    }
                }
            }
            if (state_ptr){
                state_ptr = g_slist_next (state_ptr);
                state_count++;
            }
            
            input_ptr = g_slist_next (input_ptr);
            if(input_ptr){
                input_count++;
            }

            if(input_ptr && !state_ptr){
                fprintf(stderr, " Big Errorrrrrrrrr \n");
            }
        }

        //fprintf(stdout, "Input Count : %d State Count : %d\n", input_count, state_count);
        
        if (root_state) {
            root_node_new = opttree_new_node (self);
            root_node_new->distance_from_root = 0.0;
            root_node_new->distance_from_parent = 0.0;
            root_node_new->state = optsystem_clone_state (self->optsys, root_state);
            inputs_from_parent_new = g_slist_reverse (inputs_from_parent_new);
            traj_from_parent_new = g_slist_reverse (traj_from_parent_new);
        }

        else if(!(*all_committed)){
            fprintf(stderr, "ERROR: New Root node is NULL\n");
        }
    }

    // Generate the committed trajectory
    GSList *committed_traj = NULL;
    GSList *traj_best_ptr = traj_best; 
    gboolean loop_break = FALSE;
    while (traj_best_ptr) {      

        node_t *node_this = (node_t *)(traj_best_ptr->data);
        
        // Add traj_from_parent of node_this
        GSList *state_ptr = node_this->traj_from_parent;
        while (state_ptr) {
            state_t *state_this = (state_t *)(state_ptr->data);
            committed_traj = g_slist_prepend (committed_traj, 
                                             optsystem_clone_state(self->optsys, state_this) );
            if (state_this == root_state) { //check if this is the new root node 
                loop_break = TRUE;
                break;
            }        
            state_ptr = g_slist_next (state_ptr);
        }
        if(node_this->state == root_state){ //check if this is the new root node 
            loop_break = TRUE; 
        }
        if(loop_break){
            break;
        }

        committed_traj = g_slist_prepend (committed_traj, 
                                         optsystem_clone_state (self->optsys, node_this->state));
        traj_best_ptr = g_slist_next (traj_best_ptr);
    }
    
    committed_traj = g_slist_reverse (committed_traj);
    
    // Commit to this node 
    opttree_commit_traj_with_node (self, traj_node_curr, root_node_new, traj_from_parent_new, inputs_from_parent_new);


    //fprintf (stdout, "commit traj end\n");
    
    return committed_traj;
}

void opttree_reset_old_goal_node(opttree_t *self){  
    self->lower_bound_node_to_first_goal = NULL;
}


GSList* opttree_commit_traj_to_old_goal (opttree_t *self, double time, int *all_committed) {


    if (!self->lower_bound_node_to_first_goal)
        return NULL;

    // get the best trajectory
    GSList *traj_best = NULL;
    node_t *node_curr = self->lower_bound_node_to_first_goal;
    while (node_curr) {
        traj_best = g_slist_prepend (traj_best, node_curr);
        node_curr = node_curr->parent;
    }    
    

    // find the first node that is ahead of 'time' length
    GSList *traj_ptr = traj_best;
    double time_total = 0.0;
    node_t *traj_node_curr = NULL;
    input_t *input_cut = NULL;

    traj_node_curr = (node_t *)(traj_ptr->data);

    while (traj_ptr) {

        traj_node_curr = (node_t *)(traj_ptr->data);    
        
        // Find the time to get to this node
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
        while (input_ptr) {
            input_t *input_curr = (input_t *)(input_ptr->data);
            time_total += input_curr->x[1];
            //printf ("time_total : %3.5lf \n", time_total);
            if (time_total > time) {
                input_cut = input_curr;
                break;
            }
            input_ptr = g_slist_next (input_ptr);
        }
        if (time_total >= time)
            break;
        //printf ("TIME_TOTAL : %3.5lf \n", time_total);

        traj_ptr = g_slist_next (traj_ptr);
    }

    if (time_total < time) {
        *all_committed = 1;
        //fprintf(stderr,"Remaining Path Committed\n");
    }
    if (traj_node_curr == NULL) {
        g_slist_free (traj_best);
        return NULL;
    }

    if(*all_committed){
        self->lower_bound_node_to_first_goal = NULL;
        //fprintf(stderr, " ------------ Removing Last Goal\n");
    }
    
    // Determine the root trajectory
    node_t *root_node_new = NULL;
    GSList *traj_from_parent_new = NULL;
    GSList *inputs_from_parent_new = NULL;
    state_t *root_state = NULL;
    if (input_cut) {
        GSList *state_ptr = traj_node_curr->traj_from_parent;
        GSList *input_ptr = traj_node_curr->inputs_from_parent;
        while (input_ptr) {
            state_t *state_curr = NULL;
            if(state_ptr) 
                state_curr = (state_t *)(state_ptr->data);
            input_t *input_curr = (input_t *)(input_ptr->data);
            if (input_curr == input_cut) {
                if (!state_ptr && !(*all_committed)){
                    root_state = traj_node_curr->state;//NULL;
                }
                else {
                    root_state = state_curr;
                    while (input_ptr) {
                        input_curr = (input_t *)(input_ptr->data);
                        inputs_from_parent_new = g_slist_prepend (inputs_from_parent_new, 
                                                                  optsystem_clone_input (self->optsys, input_curr));
                        input_ptr = g_slist_next (input_ptr);
                        if(state_ptr) {
                            state_curr = (state_t *)(state_ptr->data);
                            traj_from_parent_new = g_slist_prepend (traj_from_parent_new, 
                                                                    optsystem_clone_state (self->optsys, state_curr) );
                            state_ptr = g_slist_next (state_ptr);
                        }
                    }
                }
            }
            if (state_ptr)
                state_ptr = g_slist_next (state_ptr);
            input_ptr = g_slist_next (input_ptr);
        }
        
        if (root_state) {
            root_node_new = opttree_new_node (self);
            root_node_new->distance_from_root = 0.0;
            root_node_new->distance_from_parent = 0.0;
            root_node_new->state = optsystem_clone_state (self->optsys, root_state);
            inputs_from_parent_new = g_slist_reverse (inputs_from_parent_new);
            traj_from_parent_new = g_slist_reverse (traj_from_parent_new);
        }
        else if(!(*all_committed)){
            fprintf(stderr, "ERROR: New Root node is NULL\n");
        }
    }

    // Generate the committed trajectory
    GSList *committed_traj = NULL;
    GSList *traj_best_ptr = traj_best; 
    while (traj_best_ptr) {
        gboolean loop_break = FALSE;

        node_t *node_this = (node_t *)(traj_best_ptr->data);
        
        // Add traj_from_parent of node_this
        GSList *state_ptr = node_this->traj_from_parent;
        while (state_ptr) {
            state_t *state_this = (state_t *)(state_ptr->data);
            committed_traj = g_slist_prepend (committed_traj, 
                                             optsystem_clone_state(self->optsys, state_this) );
            if (state_this == root_state) {
                loop_break = TRUE;
                break;
            }
            state_ptr = g_slist_next (state_ptr);
        }
        if(node_this->state == root_state){ //check if this is the new root node 
            loop_break = TRUE; 
        }
        if(loop_break)
            break;
        committed_traj = g_slist_prepend (committed_traj, 
                                         optsystem_clone_state (self->optsys, node_this->state));
        traj_best_ptr = g_slist_next (traj_best_ptr);
    }
    committed_traj = g_slist_reverse (committed_traj);
    
    // Commit to this node 
    opttree_commit_traj_with_node (self, traj_node_curr, root_node_new, traj_from_parent_new, inputs_from_parent_new);


    return committed_traj;
}
