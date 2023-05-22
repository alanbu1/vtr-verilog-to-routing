#pragma once
#include <vector>
#include "route_tree.h"
#include "connection_based_routing.h"
#include "route_common.h"
#include "spatial_route_tree_lookup.h"
#include "timing_util.h"

/**************** Subroutines exported by route_tree_timing.cpp ***************/

RouteTree init_route_tree_to_source(ParentNetId inet);

void print_route_tree(const RouteTreeNode& rt_node);
void print_route_tree(const RouteTreeNode& rt_node, int depth);

std::tuple<vtr::optional<RouteTreeNode&>, vtr::optional<RouteTreeNode&>> update_route_tree(RouteTree& tree,
                                                                                           t_heap* hptr,
                                                                                           int target_net_pin_index,
                                                                                           SpatialRouteTreeLookup* spatial_rt_lookup,
                                                                                           bool is_flat);

void update_net_delays_from_route_tree(float* net_delay,
                                       const Netlist<>& net_list,
                                       std::vector<vtr::optional<RouteTreeNode&>>& rt_node_of_sink,
                                       ParentNetId inet,
                                       TimingInfo* timing_info,
                                       NetPinTimingInvalidator* pin_timing_invalidator);

void load_route_tree_Tdel(RouteTreeNode& rt_root, float Tarrival);
void load_route_tree_rr_route_inf(RouteTreeNode& rt_root);

bool verify_route_tree(const RouteTreeNode& rt_node);

RouteTreeNode& find_sink_rt_node(const Netlist<>& net_list, RouteTreeNode& rt_root, ParentNetId net_id, ParentPinId sink_pin);
vtr::optional<RouteTreeNode&> find_sink_rt_node_recurr(RouteTreeNode& node, RRNodeId sink_rr_inode);

/********** Incremental reroute ***********/
// instead of ripping up a net that has some congestion, cut the branches
// that don't legally lead to a sink and start routing with that partial route tree

void print_route_tree_node(const RouteTreeNode& rt_node);
void print_route_tree_inf(const RouteTreeNode& rt_node);
void print_route_tree_congestion(const RouteTreeNode& rt_node);

// Prune route tree
//
//  Note that non-configurable node will not be pruned unless the node is
//  being totally ripped up, or the node is congested.
vtr::optional<RouteTree&> prune_route_tree(RouteTree& tree, CBRR& connections_inf);

// Prune route tree
//
//  Note that non-configurable nodes will be pruned if
//  non_config_node_set_usage is provided.  prune_route_tree will update
//  non_config_node_set_usage after pruning.
vtr::optional<RouteTree&> prune_route_tree(RouteTree& tree, CBRR& connections_inf, std::vector<int>* non_config_node_set_usage);

/* Count configurable edges to non-configurable node sets. (rr_nonconf_node_sets index -> int)
 * Required when using prune_route_tree to prune non-configurable nodes. */
std::vector<int> get_non_config_node_set_usage(const RouteTree& tree);

/* Move to other pathfinder functions */
void pathfinder_update_cost_from_route_tree(const RouteTreeNode& rt_node, int add_or_sub);

bool is_equivalent_route_tree(const RouteTreeNode& rt_node1, const RouteTreeNode& rt_node2);
bool is_valid_skeleton_tree(const RouteTreeNode& rt_node);
bool is_valid_route_tree(const RouteTreeNode& rt_node);
bool is_uncongested_route_tree(const RouteTreeNode& rt_node);
float load_new_subtree_C_downstream(RouteTreeNode& rt_node);
void load_new_subtree_R_upstream(vtr::optional<RouteTreeNode&> rt_node);
