#include <cstdio>
#include <cmath>
#include <vector>

#include "rr_graph_fwd.h"
#include "rr_node_types.h"
#include "vtr_assert.h"
#include "vtr_log.h"
#include "vtr_memory.h"
#include "vtr_math.h"

#include "vpr_types.h"
#include "vpr_utils.h"
#include "vpr_error.h"

#include "globals.h"
#include "route_common.h"
#include "route_tree_timing.h"
#include "route_tree_type.h"

#include "vtr_optional.h"

/* This module keeps track of the partial routing tree for timing-driven     *
 * routing.  The normal traceback structure doesn't provide enough info      *
 * about the partial routing during timing-driven routing, so the routines   *
 * in this module are used to keep a tree representation of the partial      *
 * routing during timing-driven routing.  This allows rapid incremental      *
 * timing analysis.                                                          */

/********************** Subroutines local to this module *********************/

static std::tuple<vtr::optional<RouteTreeNode&>, vtr::optional<RouteTreeNode&>> add_subtree_to_route_tree(RouteTree& tree,
                                                                                                          t_heap* hptr,
                                                                                                          int target_net_pin_index,
                                                                                                          bool is_flat);

static void add_non_configurable_to_route_tree(RouteTreeNode& parent,
                                               const bool reached_by_non_configurable_edge,
                                               std::unordered_set<RRNodeId>& visited,
                                               bool is_flat);

static RouteTreeNode& update_unbuffered_ancestors_C_downstream(RouteTreeNode& start_of_new_subtree_rt_node);

bool verify_route_tree_recurr(const RouteTreeNode& rt_node, std::set<RRNodeId>& seen_nodes);

static vtr::optional<RouteTreeNode&> prune_route_tree_recurr(RouteTreeNode& node,
                                                             CBRR& connections_inf,
                                                             bool force_prune,
                                                             std::vector<int>* non_config_node_set_usage);

/************************** Subroutine definitions ***************************/

constexpr float epsilon = 1e-15;
static bool equal_approx(float a, float b) {
    return fabs(a - b) < epsilon;
}

/* Return a RouteTree initialized to the source of nets[inet] */
RouteTree init_route_tree_to_source(ParentNetId inet) {
    auto& route_ctx = g_vpr_ctx.routing();

    return RouteTree(RRNodeId(route_ctx.net_rr_terminals[inet][0]));
}

/* Adds the most recently finished wire segment to the routing tree, and
 * updates the Tdel, etc. numbers for the rest of the routing tree. hptr
 * is the heap pointer of the SINK that was reached, and target_net_pin_index
 * is the net pin index corresponding to the SINK that was reached. This routine
 * returns a pair: RouteTreeNode of the branch it adds to the route tree and
 * RouteTreeNode of the SINK it adds to the routing. */
std::tuple<vtr::optional<RouteTreeNode&>, vtr::optional<RouteTreeNode&>>
update_route_tree(RouteTree& tree, t_heap* hptr, int target_net_pin_index, SpatialRouteTreeLookup* spatial_rt_lookup, bool is_flat) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    float Tdel_start = 0;
    RRSwitchId iswitch;

    //Create a new subtree from the target in hptr to existing routing
    vtr::optional<RouteTreeNode&> start_of_new_subtree_rt_node, sink_rt_node;
    std::tie(start_of_new_subtree_rt_node, sink_rt_node) = add_subtree_to_route_tree(tree, hptr, target_net_pin_index, is_flat);

    if (!start_of_new_subtree_rt_node)
        return {vtr::nullopt, sink_rt_node};

    //Propagate R_upstream down into the new subtree
    load_new_subtree_R_upstream(start_of_new_subtree_rt_node.value());

    //Propagate C_downstream up from new subtree sinks to subtree root
    load_new_subtree_C_downstream(start_of_new_subtree_rt_node.value());

    //Propagate C_downstream up from the subtree root
    RouteTreeNode& unbuffered_subtree_rt_root = update_unbuffered_ancestors_C_downstream(start_of_new_subtree_rt_node.value());

    if (unbuffered_subtree_rt_root.parent) {
        RouteTreeNode& subtree_parent_rt_node = unbuffered_subtree_rt_root.parent.value();
        Tdel_start = subtree_parent_rt_node.Tdel;
        iswitch = unbuffered_subtree_rt_root.parent_switch;
        /*TODO Just a note (no action needed for this PR):In future, we need to consider APIs that returns
         * the Tdel for a routing trace in RRGraphView.*/
        Tdel_start += rr_graph.rr_switch_inf(iswitch).R * unbuffered_subtree_rt_root.C_downstream;
        Tdel_start += rr_graph.rr_switch_inf(iswitch).Tdel;
    }

    load_route_tree_Tdel(unbuffered_subtree_rt_root, Tdel_start);

    if (spatial_rt_lookup) {
        update_route_tree_spatial_lookup_recur(start_of_new_subtree_rt_node.value(), *spatial_rt_lookup);
    }

    /* if the new branch is the only child of its parent and the parent is a SOURCE,
     * it is the first time we are creating this tree, so include the parent in the new branch return
     * so that it can be included in occupancy calculation.
     * TODO: probably this should be cleaner */
    RouteTreeNode& parent = start_of_new_subtree_rt_node.value().parent.value();
    if (parent.child_nodes().size() == 1 && rr_graph.node_type(parent.inode) == SOURCE)
        return {parent, sink_rt_node};

    return {start_of_new_subtree_rt_node.value(), sink_rt_node};
}

/* Adds the most recent wire segment, ending at the SINK indicated by hptr,
 * to the routing tree. target_net_pin_index is the net pin index correspinding
 * to the SINK indicated by hptr. Returns the first (most upstream) new rt_node,
 * and the rt_node of the new SINK. Traverses up from SINK  */
static std::tuple<vtr::optional<RouteTreeNode&>, vtr::optional<RouteTreeNode&>>
add_subtree_to_route_tree(RouteTree& tree, t_heap* hptr, int target_net_pin_index, bool is_flat) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    auto& route_ctx = g_vpr_ctx.routing();

    RRNodeId sink_inode = RRNodeId(hptr->index);

    /* Walk rr_node_route_inf up until we reach an existing RouteTreeNode */
    std::vector<RRNodeId> new_branch_inodes;
    std::vector<RRSwitchId> new_branch_iswitches;

    std::unordered_set<RRNodeId> all_visited;
    std::unordered_set<RRNodeId> main_branch_visited;

    /* We need this information to build the branch:
     * node -> switch -> node -> ... -> switch -> sink.
     * Here we create two vectors:
     * new_branch_inodes: [sink, nodeN-1, nodeN-2, ... node 1] of length N
     * and new_branch_iswitches: [N-1->sink, N-2->N-1, ... 2->1, 1->found_node] of length N */
    RREdgeId edge = hptr->prev_edge();
    RRNodeId new_inode = RRNodeId(hptr->prev_node());
    RRSwitchId new_iswitch = RRSwitchId(rr_graph.rr_nodes().edge_switch(edge));

    new_branch_inodes.push_back(sink_inode);
    while (!tree.rr_node_to_rt_node.count(new_inode)) {
        new_branch_inodes.push_back(new_inode);
        new_branch_iswitches.push_back(new_iswitch);
        edge = route_ctx.rr_node_route_inf[size_t(new_inode)].prev_edge;
        new_inode = RRNodeId(route_ctx.rr_node_route_inf[size_t(new_inode)].prev_node);
        new_iswitch = RRSwitchId(rr_graph.rr_nodes().edge_switch(edge));
    }
    new_branch_iswitches.push_back(new_iswitch);

    RouteTreeNode& found_rt_node = tree.rr_node_to_rt_node.at(new_inode).value();

    /* Build a new tree branch starting from the existing node we found */
    vtr::optional<RouteTreeNode&> last_node = found_rt_node;
    all_visited.insert(last_node->inode);

    /* In the code below I'm marking SINKs and IPINs as not to be re-expanded.
     * It makes the code more efficient (though not vastly) to prune this way
     * when there aren't route-throughs or ipin doglegs.
     * ---
     * Walk through new_branch_iswitches and corresponding new_branch_inodes. */
    for (int i = new_branch_inodes.size() - 1; i >= 0; i--) {
        RouteTreeNode& new_node = last_node.value().emplace_child(new_branch_inodes[i], new_branch_iswitches[i], last_node, tree);

        e_rr_type node_type = rr_graph.node_type(new_branch_inodes[i]);
        // If is_flat is enabled, IPINs should be added, since they are used for intra-cluster routing
        if (node_type == IPIN && !is_flat) {
            new_node.re_expand = false;
        } else if (node_type == SINK) {
            new_node.re_expand = false;
            new_node.net_pin_index = target_net_pin_index; // net pin index is invalid for non-SINK nodes
        } else {
            new_node.re_expand = true;
        }

        last_node = new_node;

        main_branch_visited.insert(new_branch_inodes[i]);
        all_visited.insert(new_branch_inodes[i]);
    }

    // Expand (recursively) each of the main-branch nodes adding any
    // non-configurably connected nodes
    // Sink is not included, so no need to pass in the node's ipin value.
    for (RRNodeId rr_node : main_branch_visited) {
        add_non_configurable_to_route_tree(tree.rr_node_to_rt_node.at(rr_node).value(), false, all_visited, is_flat);
    }

    /* the first and last nodes we added.
     * vec[size-1] works, because new_branch_inodes is guaranteed to contain at least [sink, found_node] */
    vtr::optional<RouteTreeNode&> downstream_rt_node = tree.rr_node_to_rt_node.at(new_branch_inodes[new_branch_inodes.size() - 1]);
    vtr::optional<RouteTreeNode&> sink_rt_node = tree.rr_node_to_rt_node.at(new_branch_inodes[0]);

    return {downstream_rt_node, sink_rt_node};
}

static void add_non_configurable_to_route_tree(RouteTreeNode& rt_node,
                                               const bool reached_by_non_configurable_edge,
                                               std::unordered_set<RRNodeId>& visited,
                                               bool is_flat) {
    if (visited.count(rt_node.inode) && reached_by_non_configurable_edge)
        return;

    visited.insert(rt_node.inode);

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    VTR_ASSERT(rt_node.root);
    auto& rr_node_to_rt_node = rt_node.root.value().rr_node_to_rt_node;
    const RRNodeId rr_node = rt_node.inode;

    for (int iedge : rr_graph.non_configurable_edges(rr_node)) {
        // Recursive case: expand children
        VTR_ASSERT(!rr_graph.edge_is_configurable(rr_node, iedge));
        RRNodeId to_rr_node = rr_graph.edge_sink_node(rr_node, iedge);

        if (rr_node_to_rt_node.count(to_rr_node)) // TODO: not 100% sure about this
            continue;

        RRSwitchId edge_switch(rr_graph.edge_switch(rr_node, iedge));

        RouteTreeNode& new_node = rt_node.emplace_child_front(to_rr_node, edge_switch, rt_node, rt_node.root);
        new_node.net_pin_index = OPEN;
        if (rr_graph.node_type(RRNodeId(to_rr_node)) == IPIN && !is_flat) {
            new_node.re_expand = false;
        } else {
            new_node.re_expand = true;
        }

        add_non_configurable_to_route_tree(new_node, true, visited, is_flat);
    }
}

void load_new_subtree_R_upstream(vtr::optional<RouteTreeNode&> rt_node) {
    /* Sets the R_upstream values of all the nodes in the new path to the
     * correct value by traversing down to SINK from the start of the new path. */

    if (!rt_node) {
        return;
    }

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    vtr::optional<RouteTreeNode&> parent_rt_node = rt_node.value().parent;
    RRNodeId inode = rt_node.value().inode;

    //Calculate upstream resistance
    float R_upstream = 0.;
    if (parent_rt_node) {
        RRSwitchId iswitch = rt_node.value().parent_switch;
        bool switch_buffered = rr_graph.rr_switch_inf(iswitch).buffered();

        if (!switch_buffered) {
            R_upstream += parent_rt_node.value().R_upstream; //Parent upstream R
        }
        R_upstream += rr_graph.rr_switch_inf(iswitch).R; //Parent switch R
    }
    R_upstream += rr_graph.node_R(inode); //Current node R

    rt_node.value().R_upstream = R_upstream;

    //Update children
    for (auto& child : rt_node.value().child_nodes()) {
        load_new_subtree_R_upstream(child);
    }
}

float load_new_subtree_C_downstream(RouteTreeNode& rt_node) {
    float C_downstream = 0.;

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    C_downstream += rr_graph.node_C(RRNodeId(rt_node.inode));
    for (auto& child : rt_node.child_nodes()) {
        /*Similar to net_delay.cpp, this for loop traverses a rc subtree, whose edges represent enabled switches.
         * When switches such as multiplexers and tristate buffers are enabled, their fanout
         * produces an "internal capacitance". We account for this internal capacitance of the
         * switch by adding it to the total capacitance of the node.*/
        C_downstream += rr_graph.rr_switch_inf(child.parent_switch).Cinternal;
        float C_downstream_child = load_new_subtree_C_downstream(child);
        if (!rr_graph.rr_switch_inf(child.parent_switch).buffered()) {
            C_downstream += C_downstream_child;
        }
    }

    rt_node.C_downstream = C_downstream;
    return C_downstream;
}

static RouteTreeNode&
update_unbuffered_ancestors_C_downstream(RouteTreeNode& rt_node) {
    /* Updates the C_downstream values for the ancestors of the new path.  Once
     * a buffered switch is found amongst the ancestors, no more ancestors are
     * affected.  Returns the root of the "unbuffered subtree" whose Tdel
     * values are affected by the new path's addition.                          */
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    RRSwitchId iswitch = rt_node.parent_switch;

    /* Now that a connection has been made between rt_node and its parent we must also consider
     * the potential effect of internal capacitance. We will first assume that parent is connected
     * by an unbuffered switch, so the ancestors downstream capacitance must be equal to the sum
     * of the child's downstream capacitance with the internal capacitance of the switch.*/

    float C_downstream_addition = rt_node.C_downstream + rr_graph.rr_switch_inf(iswitch).Cinternal;

    /* Having set the value of C_downstream_addition, we must check whether the parent switch
     * is a buffered or unbuffered switch with the if statement below. If the parent switch is
     * a buffered switch, then the parent node's downsteam capacitance is increased by the
     * value of the parent switch's internal capacitance in the if statement below.
     * Correspondingly, the ancestors' downstream capacitance will be updated by the same
     * value through the while loop. Otherwise, if the parent switch is unbuffered, then
     * the if statement will be ignored, and the parent and ancestral nodes' downstream
     * capacitance will be increased by the sum of the child's downstream capacitance with
     * the internal capacitance of the parent switch in the while loop.*/

    vtr::optional<RouteTreeNode&> last_node = rt_node;
    vtr::optional<RouteTreeNode&> parent_rt_node = rt_node.parent;

    if (parent_rt_node && rr_graph.rr_switch_inf(iswitch).buffered() == true) {
        C_downstream_addition = rr_graph.rr_switch_inf(iswitch).Cinternal;
        last_node = parent_rt_node;
        last_node.value().C_downstream += C_downstream_addition;
        parent_rt_node = last_node.value().parent;
        iswitch = last_node.value().parent_switch;
    }

    while (parent_rt_node && rr_graph.rr_switch_inf(iswitch).buffered() == false) {
        last_node = parent_rt_node;
        last_node.value().C_downstream += C_downstream_addition;
        parent_rt_node = last_node.value().parent;
        iswitch = last_node.value().parent_switch;
    }

    VTR_ASSERT(last_node);
    return last_node.value();
}

void load_route_tree_Tdel(RouteTreeNode& rt_node, float Tarrival) {
    /* Updates the Tdel values of the subtree rooted at rt_node by
     * by calling itself recursively.  The C_downstream values of all the nodes
     * must be correct before this routine is called.  Tarrival is the time at
     * at which the signal arrives at this node's *input*.                      */
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    RRNodeId inode = rt_node.inode;
    RRSwitchId iswitch;
    float Tdel, Tchild;

    /* Assuming the downstream connections are, on average, connected halfway
     * along a wire segment's length.  See discussion in net_delay.c if you want
     * to change this.                                                           */
    Tdel = Tarrival + 0.5 * rt_node.C_downstream * rr_graph.node_R(RRNodeId(inode));
    rt_node.Tdel = Tdel;

    /* Now expand the children of this node to load their Tdel values (depth-
     * first pre-order traversal).                                              */
    for (auto& child : rt_node.child_nodes()) {
        iswitch = child.parent_switch;

        Tchild = Tdel + rr_graph.rr_switch_inf(RRSwitchId(iswitch)).R * child.C_downstream;
        Tchild += rr_graph.rr_switch_inf(RRSwitchId(iswitch)).Tdel; /* Intrinsic switch delay. */
        load_route_tree_Tdel(child, Tchild);
    }
}

void load_route_tree_rr_route_inf(RouteTreeNode& rt_node) {
    /* Traverses down a route tree and updates rr_node_inf for all nodes
     * to reflect that these nodes have already been routed to */
    auto& route_ctx = g_vpr_ctx.mutable_routing();

    for (auto& child : rt_node.child_nodes()) {
        RRNodeId inode = child.inode;
        route_ctx.rr_node_route_inf[size_t(inode)].prev_node = NO_PREVIOUS;
        route_ctx.rr_node_route_inf[size_t(inode)].prev_edge = RREdgeId::INVALID();

        // path cost should be unset
        VTR_ASSERT(std::isinf(route_ctx.rr_node_route_inf[size_t(inode)].path_cost));
        VTR_ASSERT(std::isinf(route_ctx.rr_node_route_inf[size_t(inode)].backward_path_cost));

        load_route_tree_rr_route_inf(child);
    }
}

bool verify_route_tree(const RouteTreeNode& rt_root) {
    std::set<RRNodeId> seen_nodes;
    return verify_route_tree_recurr(rt_root, seen_nodes);
}

bool verify_route_tree_recurr(const RouteTreeNode& rt_node, std::set<RRNodeId>& seen_nodes) {
    if (seen_nodes.count(rt_node.inode)) {
        VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Duplicate route tree nodes found for node %d", rt_node.inode);
    }
    seen_nodes.insert(rt_node.inode);

    for (auto& child : rt_node.child_nodes()) {
        verify_route_tree_recurr(child, seen_nodes);
    }

    return true;
}

void print_route_tree(const RouteTreeNode& rt_node) {
    print_route_tree(rt_node, 0);
}

void print_route_tree(const RouteTreeNode& rt_node, int depth) {
    std::string indent;
    for (int i = 0; i < depth; ++i) {
        indent += "  ";
    }

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    VTR_LOG("%srt_node: %d (%s) \t ipin: %d \t R: %g \t C: %g \t delay: %g \t",
            indent.c_str(),
            rt_node.inode,
            rr_graph.node_type_string(RRNodeId(rt_node.inode)),
            rt_node.net_pin_index,
            rt_node.R_upstream,
            rt_node.C_downstream,
            rt_node.Tdel);

    if (rt_node.parent) {
        VTR_LOG("parent: %d \t parent_switch: %d", rt_node.parent.value().inode, rt_node.parent_switch);
        bool parent_edge_configurable = rr_graph.rr_switch_inf(rt_node.parent_switch).configurable();
        if (!parent_edge_configurable) {
            VTR_LOG("*");
        }
    }

    auto& route_ctx = g_vpr_ctx.routing();
    if (route_ctx.rr_node_route_inf[size_t(rt_node.inode)].occ() > rr_graph.node_capacity(rt_node.inode)) {
        VTR_LOG(" x");
    }

    VTR_LOG("\n");

    for (auto& child : rt_node.child_nodes()) {
        print_route_tree(child, depth + 1);
    }
}

void update_net_delays_from_route_tree(float* net_delay,
                                       const Netlist<>& net_list,
                                       std::vector<vtr::optional<RouteTreeNode&>>& rt_node_of_sink,
                                       ParentNetId inet,
                                       TimingInfo* timing_info,
                                       NetPinTimingInvalidator* pin_timing_invalidator) {
    /* Goes through all the sinks of this net and copies their delay values from
     * the route_tree to the net_delay array.                                    */
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    for (unsigned int isink = 1; isink < net_list.net_pins(inet).size(); isink++) {
        VTR_ASSERT(rr_graph.node_type(rt_node_of_sink[isink].value().inode) == SINK);
        float new_delay = rt_node_of_sink[isink]->Tdel;

        if (pin_timing_invalidator && new_delay != net_delay[isink]) {
            //Delay changed, invalidate for incremental timing update
            VTR_ASSERT_SAFE(timing_info);
            ParentPinId pin = net_list.net_pin(inet, isink);
            pin_timing_invalidator->invalidate_connection(pin, timing_info);
        }

        net_delay[isink] = new_delay;
    }
}

//Prunes a route tree (recursively) based on congestion and the 'force_prune' argument
//
//Returns true if the current node was pruned
static vtr::optional<RouteTreeNode&> prune_route_tree_recurr(RouteTreeNode& node, CBRR& connections_inf, bool force_prune, std::vector<int>* non_config_node_set_usage) {
    // Recursively traverse the route tree rooted at node and remove any congested
    // sub-trees
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    auto& route_ctx = g_vpr_ctx.routing();
    bool congested = (route_ctx.rr_node_route_inf[size_t(node.inode)].occ() > rr_graph.node_capacity(node.inode));

    int node_set = -1;
    auto itr = device_ctx.rr_node_to_non_config_node_set.find(size_t(node.inode));
    if (itr != device_ctx.rr_node_to_non_config_node_set.end()) {
        node_set = itr->second;
    }

    if (congested) {
        //This connection is congested -- prune it
        force_prune = true;
    }

    if (connections_inf.should_force_reroute_connection(size_t(node.inode))) {
        //Forcibly re-route (e.g. to improve delay)
        force_prune = true;
    }

    // Recursively prune child nodes
    bool all_children_pruned = true;
    node.remove_child_if([&](auto& child) {
        vtr::optional<RouteTreeNode&> child_maybe = prune_route_tree_recurr(child,
                                                                            connections_inf,
                                                                            force_prune,
                                                                            non_config_node_set_usage);

        if (child_maybe.has_value()) { // Not pruned
            all_children_pruned = false;
            return false;
        } else { // Pruned
            // After removing a child node, check if non_config_node_set_usage
            // needs an update.
            if (non_config_node_set_usage != nullptr && node_set != -1 && rr_graph.rr_switch_inf(child.parent_switch).configurable()) {
                (*non_config_node_set_usage)[node_set] -= 1;
                VTR_ASSERT((*non_config_node_set_usage)[node_set] >= 0);
            }
            return true;
        }
    });

    if (rr_graph.node_type(node.inode) == SINK) {
        if (!force_prune) {
            //Valid path to sink

            //Record sink as reachable
            connections_inf.reached_rt_sink(node.inode);

            return node; // Not pruned
        } else {
            //Record as not reached
            connections_inf.toreach_rr_sink(node.net_pin_index);

            return vtr::nullopt; // Pruned
        }
    } else if (all_children_pruned) {
        //This node has no children
        //
        // This can happen in three scenarios:
        //   1) This node is being pruned. As a result any child nodes
        //      (subtrees) will have been pruned.
        //
        //   2) This node was reached by a non-configurable edge but
        //      was otherwise unused (forming a 'stub' off the main
        //      branch).
        //
        //   3) This node is uncongested, but all its connected sub-trees
        //      have been pruned.
        //
        // We take the corresponding actions:
        //   1) Prune the node.
        //
        //   2) Prune the node only if the node set is unused or if force_prune
        //      is set.
        //
        //   3) Prune the node.
        //
        //      This avoids the creation of unused 'stubs'. (For example if
        //      we left the stubs in and they were subsequently not used
        //      they would uselessly consume routing resources).
        VTR_ASSERT(node.child_nodes().size() == 0);

        bool reached_non_configurably = false;
        if (node.parent) {
            reached_non_configurably = !rr_graph.rr_switch_inf(node.parent_switch).configurable();

            if (reached_non_configurably) {
                // Check if this non-configurable node set is in use.
                VTR_ASSERT(node_set != -1);
                if (non_config_node_set_usage != nullptr && (*non_config_node_set_usage)[node_set] == 0) {
                    force_prune = true;
                }
            }
        }

        if (reached_non_configurably && !force_prune) {
            return node; //Not pruned
        } else {
            return vtr::nullopt; //Pruned
        }

    } else {
        // If this node is:
        //   1. Part of a non-configurable node set
        //   2. The first node in the tree that is part of the non-configurable
        //      node set
        //
        //      -- and --
        //
        //   3. The node set is not active
        //
        //  Then prune this node.
        //
        if (non_config_node_set_usage != nullptr && node_set != -1 && rr_graph.rr_switch_inf(node.parent_switch).configurable() && (*non_config_node_set_usage)[node_set] == 0) {
            // This node should be pruned, re-prune edges once more.
            //
            // If the following is true:
            //
            //  - The node set is unused
            //    (e.g. (*non_config_node_set_usage)[node_set] == 0)
            //  - This particular node still had children
            //    (which is true by virtue of being in this else statement)
            //
            // Then that indicates that the node set became unused during the
            // pruning. One or more of the children of this node will be
            // pruned if prune_route_tree_recurr is called again, and
            // eventually the whole node will be prunable.
            //
            //  Consider the following graph:
            //
            //  1 -> 2
            //  2 -> 3 [non-configurable]
            //  2 -> 4 [non-configurable]
            //  3 -> 5
            //  4 -> 6
            //
            //  Assume that nodes 5 and 6 do not connect to a sink, so they
            //  will be pruned (as normal). When prune_route_tree_recurr
            //  visits 2 for the first time, node 3 or 4 will remain. This is
            //  because when prune_route_tree_recurr visits 3 or 4, it will
            //  not have visited 4 or 3 (respectively). As a result, either
            //  node 3 or 4 will not be pruned on the first pass, because the
            //  node set usage count will be > 0. However after
            //  prune_route_tree_recurr visits 2, 3 and 4, the node set usage
            //  will be 0, so everything can be pruned.
            return prune_route_tree_recurr(node, connections_inf,
                                           /*force_prune=*/false, non_config_node_set_usage);
        }

        //An unpruned intermediate node
        VTR_ASSERT(!force_prune);

        return node; //Not pruned
    }
}

vtr::optional<RouteTree&> prune_route_tree(RouteTree& tree, CBRR& connections_inf) {
    return prune_route_tree(tree, connections_inf, nullptr);
}

vtr::optional<RouteTree&> prune_route_tree(RouteTree& tree, CBRR& connections_inf, std::vector<int>* non_config_node_set_usage) {
    /* Prune a skeleton route tree of illegal branches - when there is at least 1 congested node on the path to a sink
     * This is the top level function to be called with the SOURCE node as root.
     * Returns nullopt if the entire tree has been pruned.
     *
     * Note: does not update R_upstream/C_downstream
     */

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    auto& route_ctx = g_vpr_ctx.routing();

    VTR_ASSERT_MSG(rr_graph.node_type(tree.root.inode) == SOURCE, "Root of route tree must be SOURCE");

    VTR_ASSERT_MSG(route_ctx.rr_node_route_inf[size_t(tree.root.inode)].occ() <= rr_graph.node_capacity(tree.root.inode),
                   "Route tree root/SOURCE should never be congested");

    auto pruned_node = prune_route_tree_recurr(tree.root, connections_inf, false, non_config_node_set_usage);
    if (pruned_node)
        return tree;
    else
        return vtr::nullopt;
}

/* For each non-configurable node set, count the cases in the route tree where:
 * 1. the node is a member of a nonconf set (duh)
 * if not SINK:
 *   2. and there *is* an outgoing edge (we are not at the end of a stub)
 *   3. and that outgoing edge is a configurable switch
 * if SINK:
 *   2. and the incoming edge is a configurable switch
 *     (Note: the old code's comments mention that a "non-configurable edge"
 *      "to" a sink is a usage of the set, but the code used to check if the
 *      edge "from" the SINK, which shouldn't exist, was "configurable". This
 *      might be some faulty code carried over.) */
std::vector<int> get_non_config_node_set_usage(const RouteTree& tree) {
    auto& device_ctx = g_vpr_ctx.device();
    std::vector<int> usage(device_ctx.rr_non_config_node_sets.size(), 0);

    const auto& rr_to_nonconf = device_ctx.rr_node_to_non_config_node_set;

    for (auto& rt_node : tree) {
        auto it = rr_to_nonconf.find(size_t(rt_node.inode));
        if (it == rr_to_nonconf.end())
            continue;

        if (device_ctx.rr_graph.node_type(rt_node.inode) == SINK) {
            if (device_ctx.rr_graph.rr_switch_inf(rt_node.parent_switch).configurable()) {
                usage[it->second] += 1;
            }
            continue;
        }

        if (rt_node.child_nodes().empty())
            continue;

        for (auto& child : rt_node.child_nodes()) {
            if (device_ctx.rr_graph.rr_switch_inf(child.parent_switch).configurable()) {
                usage[it->second] += 1;
            }
        }
    }

    return usage;
}

void pathfinder_update_cost_from_route_tree(const RouteTreeNode& rt_node, int add_or_sub) {
    /* Update pathfinder cost of all nodes rooted at rt_node, including rt_node itself */
    pathfinder_update_single_node_occupancy(size_t(rt_node.inode), add_or_sub);

    for (auto& child : rt_node.child_nodes()) {
        pathfinder_update_cost_from_route_tree(child, add_or_sub);
    }
}

/***************** Debugging and printing for incremental rerouting ****************/
template<typename Op>
static void traverse_indented_route_tree(const RouteTreeNode& rt_node, int branch_level, bool new_branch, Op op, int indent_level) {
    /* pretty print the route tree; what's printed depends on the printer Op passed in */

    // rely on preorder depth first traversal
    // print branch indent
    if (new_branch) VTR_LOG("\n%*s", indent_level * branch_level, " \\ ");

    op(rt_node);
    // reached sink, move onto next branch
    if (!rt_node.child_nodes().size()) return;
    // branch point, has sibling edge
    else if (rt_node.child_nodes().size() > 1) {
        bool first_branch = true;
        for (auto& child : rt_node.child_nodes()) {
            // don't print a new line for the first branch
            traverse_indented_route_tree(child, branch_level + 1, !first_branch, op, indent_level);
            first_branch = false;
        }
    }
    // along a path, just propagate down
    else {
        traverse_indented_route_tree(rt_node.child_nodes().back(), branch_level + 1, false, op, indent_level);
    }
}

static void print_node(const RouteTreeNode& rt_node) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    RRNodeId inode = rt_node.inode;
    t_rr_type node_type = rr_graph.node_type(inode);
    VTR_LOG("%5.1e %5.1e %2d%6s|%-6d-> ", rt_node.C_downstream, rt_node.R_upstream,
            rt_node.re_expand, rr_node_typename[node_type], inode);
}

static void print_node_inf(const RouteTreeNode& rt_node) {
    auto& route_ctx = g_vpr_ctx.routing();

    RRNodeId inode = rt_node.inode;
    const auto& node_inf = route_ctx.rr_node_route_inf[size_t(inode)];
    VTR_LOG("%5.1e %5.1e%6d%3d|%-6d-> ", node_inf.path_cost, node_inf.backward_path_cost,
            node_inf.prev_node, node_inf.prev_edge, inode);
}

static void print_node_congestion(const RouteTreeNode& rt_node) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    auto& route_ctx = g_vpr_ctx.routing();

    RRNodeId inode = rt_node.inode;
    const auto& node_inf = route_ctx.rr_node_route_inf[size_t(inode)];
    const short& node_capacity = rr_graph.node_capacity(RRNodeId(inode));
    VTR_LOG("%2d %2d|%-6d-> ", node_inf.acc_cost, rt_node.Tdel,
            node_inf.occ(), node_capacity, inode);
}

void print_route_tree_inf(const RouteTreeNode& rt_node) {
    traverse_indented_route_tree(rt_node, 0, false, print_node_inf, 34);
    VTR_LOG("\n");
}

void print_route_tree_node(const RouteTreeNode& rt_node) {
    traverse_indented_route_tree(rt_node, 0, false, print_node, 34);
    VTR_LOG("\n");
}

void print_route_tree_congestion(const RouteTreeNode& rt_node) {
    traverse_indented_route_tree(rt_node, 0, false, print_node_congestion, 15);
    VTR_LOG("\n");
}

/* the following is_* functions are for debugging correctness of pruned route tree
 * these should only be called when the debug switch DEBUG_INCREMENTAL_REROUTING is on */
bool is_equivalent_route_tree(const RouteTreeNode& rt_node1, const RouteTreeNode& rt_node2) {
    if ((rt_node1.inode != rt_node2.inode) || (rt_node1.parent_switch != rt_node2.parent_switch) || (!equal_approx(rt_node1.R_upstream, rt_node2.R_upstream)) || (!equal_approx(rt_node1.C_downstream, rt_node2.C_downstream)) || (!equal_approx(rt_node1.Tdel, rt_node2.Tdel))) {
        VTR_LOG("mismatch i %d|%d s %d|%d R %e|%e C %e|%e T %e %e\n",
                rt_node1.inode, rt_node2.inode,
                rt_node1.parent_switch, rt_node2.parent_switch,
                rt_node1.R_upstream, rt_node2.R_upstream,
                rt_node1.C_downstream, rt_node2.C_downstream,
                rt_node1.Tdel, rt_node2.Tdel);
        return false;
    }

    if (rt_node1.child_nodes().size() != rt_node2.child_nodes().size()) {
        VTR_LOG("one of the trees have an extra edge!\n");
        return false;
    }

    /* zip-iterate both child_nodes lists and check equivalence */
    auto it1 = rt_node1.child_nodes().begin();
    auto it2 = rt_node2.child_nodes().begin();
    for (;
         it1 != rt_node1.child_nodes().end() && it2 != rt_node2.child_nodes().end();
         it1++, it2++) {
        if (!is_equivalent_route_tree(*it1, *it2))
            return false; // child trees not equivalent
    }

    return true; // passed all tests
}

// check only the connections are correct, ignore R and C
bool is_valid_skeleton_tree(const RouteTreeNode& rt_node) {
    RRNodeId inode = rt_node.inode;
    for (auto& child : rt_node.child_nodes()) {
        if (child.parent != rt_node) {
            VTR_LOG("parent-child relationship not mutually acknowledged by parent %d->%d child %d<-%d\n",
                    inode, child.inode,
                    child.inode, child.parent.value().inode);
            return false;
        }

        if (!is_valid_skeleton_tree(child)) {
            VTR_LOG("subtree %d invalid, propagating up\n", child.inode);
            return false;
        }
    }
    return true;
}

bool is_valid_route_tree(const RouteTreeNode& rt_node) {
    // check upstream resistance
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    auto& route_ctx = g_vpr_ctx.routing();

    constexpr float CAP_REL_TOL = 1e-6;
    constexpr float CAP_ABS_TOL = vtr::DEFAULT_ABS_TOL;
    constexpr float RES_REL_TOL = 1e-6;
    constexpr float RES_ABS_TOL = vtr::DEFAULT_ABS_TOL;

    RRNodeId inode = rt_node.inode;
    RRSwitchId iswitch = rt_node.parent_switch;
    if (rt_node.parent) {
        if (rr_graph.rr_switch_inf(iswitch).buffered()) {
            float R_upstream_check = rr_graph.node_R(inode) + rr_graph.rr_switch_inf(iswitch).R;
            if (!vtr::isclose(rt_node.R_upstream, R_upstream_check, RES_REL_TOL, RES_ABS_TOL)) {
                VTR_LOG("%d mismatch R upstream %e supposed %e\n", inode, rt_node.R_upstream, R_upstream_check);
                return false;
            }
        } else {
            float R_upstream_check = rr_graph.node_R(inode) + rt_node.parent.value().R_upstream + rr_graph.rr_switch_inf(iswitch).R;
            if (!vtr::isclose(rt_node.R_upstream, R_upstream_check, RES_REL_TOL, RES_ABS_TOL)) {
                VTR_LOG("%d mismatch R upstream %e supposed %e\n", inode, rt_node.R_upstream, R_upstream_check);
                return false;
            }
        }
    } else if (rt_node.R_upstream != rr_graph.node_R(inode)) {
        VTR_LOG("%d mismatch R upstream %e supposed %e\n", inode, rt_node.R_upstream, rr_graph.node_R(inode));
        return false;
    }

    if (rt_node.child_nodes().size() == 0) { // sink, must not be congested
        int occ = route_ctx.rr_node_route_inf[size_t(inode)].occ();
        int capacity = rr_graph.node_capacity(inode);
        if (occ > capacity) {
            VTR_LOG("SINK %d occ %d > cap %d\n", inode, occ, capacity);
            return false;
        }
    }

    // check downstream C
    float C_downstream_children = 0;
    for (auto& child : rt_node.child_nodes()) {
        if (child.parent != rt_node) {
            VTR_LOG("parent-child relationship not mutually acknowledged by parent %d->%d child %d<-%d\n",
                    inode, child.inode,
                    child.inode, rt_node.inode);
            return false;
        }
        C_downstream_children += rr_graph.rr_switch_inf(child.parent_switch).Cinternal;

        if (!rr_graph.rr_switch_inf(child.parent_switch).buffered()) {
            C_downstream_children += child.C_downstream;
        }
        if (!is_valid_route_tree(child)) {
            VTR_LOG("subtree %d invalid, propagating up\n", child.inode);
            return false;
        }
    }

    float C_downstream_check = C_downstream_children + rr_graph.node_C(inode);
    if (!vtr::isclose(rt_node.C_downstream, C_downstream_check, CAP_REL_TOL, CAP_ABS_TOL)) {
        VTR_LOG("%d mismatch C downstream %e supposed %e\n", inode, rt_node.C_downstream, C_downstream_check);
        return false;
    }

    return true;
}

//Returns true if the route tree rooted at 'root' is not congested
bool is_uncongested_route_tree(const RouteTreeNode& rt_node) {
    auto& route_ctx = g_vpr_ctx.routing();
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    RRNodeId inode = rt_node.inode;
    if (route_ctx.rr_node_route_inf[size_t(inode)].occ() > rr_graph.node_capacity(RRNodeId(inode))) {
        //This node is congested
        return false;
    }

    for (auto& child : rt_node.child_nodes()) {
        if (!is_uncongested_route_tree(child)) {
            // The sub-tree connected to this edge is congested
            return false;
        }
    }

    // The sub-tree below the current node is uncongested
    return true;
}

RouteTreeNode& find_sink_rt_node(const Netlist<>& net_list, RouteTreeNode& rt_root, ParentNetId net_id, ParentPinId sink_pin) {
    //Given the net_id and the sink_pin, this two-step function finds a pointer to the
    //route tree sink corresponding to sink_pin. This function constitutes the first step,
    //in which, we loop through the pins of the net and terminate the search once the mapping
    //of (net_id, ipin) -> sink_pin is found. Conveniently, the pair (net_id, ipin) can
    //be further translated to the index of the routing resource node sink_rr_inode.
    //In the second step, we pass the root of the route tree and sink_rr_inode in order to
    //recursively traverse the route tree until we reach the sink node that corresponds
    //to sink_rr_inode.

    auto& route_ctx = g_vpr_ctx.routing();

    int ipin = net_list.pin_net_index(sink_pin);
    RRNodeId sink_rr_inode = RRNodeId(route_ctx.net_rr_terminals[net_id][ipin]); //obtain the value of the routing resource sink

    vtr::optional<RouteTreeNode&> sink_rt_node = find_sink_rt_node_recurr(rt_root, sink_rr_inode);
    VTR_ASSERT(sink_rt_node.has_value());
    return sink_rt_node.value();
}

vtr::optional<RouteTreeNode&> find_sink_rt_node_recurr(RouteTreeNode& node, RRNodeId sink_rr_inode) {
    if (node.inode == sink_rr_inode) { //check if current node matches sink_rr_inode
        return node;
    }

    for (auto& child : node.child_nodes()) {
        vtr::optional<RouteTreeNode&> found_node = find_sink_rt_node_recurr(child, sink_rr_inode); // process each of the children
        if (found_node && found_node.value().inode == sink_rr_inode) {
            // If the sink has been found downstream in the branch, we would like to immediately exit the search
            return found_node;
        }
    }
    return vtr::nullopt; // We have not reached the sink node
}
