#include "old_traceback.h"

#include "clustered_netlist_fwd.h"
#include "globals.h"
#include "vtr_assert.h"

#include <vector>

std::pair<t_trace*, t_trace*> traceback_from_route_tree_recurr(t_trace* head, t_trace* tail, const RouteTreeNode& node);
static void traceback_to_route_tree_x(
    std::unordered_map<RRNodeId, vtr::optional<RouteTreeNode&>> rr_node_to_rt_node,
    t_trace* trace,
    RouteTreeNode& parent,
    RRSwitchId parent_switch);

bool validate_traceback_recurr(t_trace* trace, std::set<int>& seen_rr_nodes);
void free_trace_data(t_trace* tptr);

/* Builds a skeleton route tree from a traceback
 * does not calculate R_upstream, C_downstream, or Tdel (left uninitialized)
 * returns the root of the converted route tree */
vtr::optional<RouteTree> traceback_to_route_tree(t_trace* head) {
    if (head == nullptr)
        return vtr::nullopt;

    RouteTreeNode root(RRNodeId(head->index), RRSwitchId(OPEN), vtr::nullopt);
    std::unordered_map<RRNodeId, vtr::optional<RouteTreeNode&>> rr_node_to_rt_node;

    rr_node_to_rt_node[RRNodeId(head->index)] = root;

    if (head->next)
        traceback_to_route_tree_x(rr_node_to_rt_node, head->next, root, RRSwitchId(head->iswitch));

    RouteTree tree(std::move(root));
    tree.reload_timing();

    /* We built the tree using an external lookup, but returning by value should either copy
     * or move, which will make the tree update its own lookup */
    return tree;
}

/* Add the path indicated by the trace to parent */
static void traceback_to_route_tree_x(std::unordered_map<RRNodeId, vtr::optional<RouteTreeNode&>> rr_node_to_rt_node, t_trace* trace, RouteTreeNode& parent, RRSwitchId parent_switch) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    RRNodeId inode = RRNodeId(trace->index);

    RouteTreeNode new_node = parent.emplace_child(inode, parent_switch, parent);
    new_node.net_pin_index = trace->net_pin_index;
    new_node.R_upstream = std::numeric_limits<float>::quiet_NaN();
    new_node.C_downstream = std::numeric_limits<float>::quiet_NaN();
    new_node.Tdel = std::numeric_limits<float>::quiet_NaN();
    auto node_type = rr_graph.node_type(inode);
    if (node_type == IPIN || node_type == SINK)
        new_node.re_expand = false;
    else
        new_node.re_expand = true;

    rr_node_to_rt_node[new_node.inode] = new_node;

    if (rr_graph.node_type(inode) == SINK) {
        /* The traceback returns to the previous branch point if there is more than one SINK, else we are at the base case */
        if (trace->next) {
            RRNodeId next_rr_node = RRNodeId(trace->next->index);
            RouteTreeNode& branch = rr_node_to_rt_node.at(next_rr_node).value();
            VTR_ASSERT(trace->next->next);
            traceback_to_route_tree_x(rr_node_to_rt_node, trace->next->next, branch, RRSwitchId(trace->next->iswitch));
        }
    } else {
        traceback_to_route_tree_x(rr_node_to_rt_node, trace->next, new_node, RRSwitchId(trace->iswitch));
    }
}

std::pair<t_trace*, t_trace*> traceback_from_route_tree_recurr(t_trace* head, t_trace* tail, const RouteTreeNode& node) {
    if (node.child_nodes().size() > 0) {
        //Recursively add children
        for (auto& child : node.child_nodes()) {
            t_trace* curr = alloc_trace_data();
            curr->index = size_t(node.inode);
            curr->net_pin_index = node.net_pin_index;
            curr->iswitch = size_t(child.parent_switch);
            curr->next = nullptr;

            if (tail) {
                VTR_ASSERT(tail->next == nullptr);
                tail->next = curr;
            }

            tail = curr;

            if (!head) {
                head = tail;
            }

            std::tie(head, tail) = traceback_from_route_tree_recurr(head, tail, child);
        }
    } else {
        //Leaf
        t_trace* curr = alloc_trace_data();
        curr->index = size_t(node.inode);
        curr->net_pin_index = node.net_pin_index;
        curr->iswitch = OPEN;
        curr->next = nullptr;

        if (tail) {
            VTR_ASSERT(tail->next == nullptr);
            tail->next = curr;
        }

        tail = curr;

        if (!head) {
            head = tail;
        }
    }

    return {head, tail};
}

/* Creates a traceback from the route tree */
t_trace* traceback_from_route_tree(const RouteTree& tree) {
    t_trace* head;
    t_trace* tail;

    std::tie(head, tail) = traceback_from_route_tree_recurr(nullptr, nullptr, tree.root());

    VTR_ASSERT(validate_traceback(head));

    return head;
}

void print_traceback(const t_trace* trace) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    auto& route_ctx = g_vpr_ctx.routing();
    const t_trace* prev = nullptr;
    while (trace) {
        int inode = trace->index;
        VTR_LOG("%d (%s)", inode, rr_node_typename[rr_graph.node_type(RRNodeId(inode))]);

        if (trace->iswitch == OPEN) {
            VTR_LOG(" !"); //End of branch
        }

        if (prev && prev->iswitch != OPEN && !rr_graph.rr_switch_inf(RRSwitchId(prev->iswitch)).configurable()) {
            VTR_LOG("*"); //Reached non-configurably
        }

        if (route_ctx.rr_node_route_inf[inode].occ() > rr_graph.node_capacity(RRNodeId(inode))) {
            VTR_LOG(" x"); //Overused
        }
        VTR_LOG("\n");
        prev = trace;
        trace = trace->next;
    }
    VTR_LOG("\n");
}

bool validate_traceback(t_trace* trace) {
    std::set<int> seen_rr_nodes;

    return validate_traceback_recurr(trace, seen_rr_nodes);
}

bool validate_traceback_recurr(t_trace* trace, std::set<int>& seen_rr_nodes) {
    if (!trace) {
        return true;
    }

    seen_rr_nodes.insert(trace->index);

    t_trace* next = trace->next;

    if (next) {
        if (trace->iswitch == OPEN) { //End of a branch

            //Verify that the next element (branch point) has been already seen in the traceback so far
            if (!seen_rr_nodes.count(next->index)) {
                VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Traceback branch point %d not found", next->index);
            } else {
                //Recurse along the new branch
                return validate_traceback_recurr(next, seen_rr_nodes);
            }
        } else { //Midway along branch

            //Check there is an edge connecting trace and next

            auto& device_ctx = g_vpr_ctx.device();
            const auto& rr_graph = device_ctx.rr_graph;
            bool found = false;
            for (t_edge_size iedge = 0; iedge < rr_graph.num_edges(RRNodeId(trace->index)); ++iedge) {
                int to_node = size_t(rr_graph.edge_sink_node(RRNodeId(trace->index), iedge));

                if (to_node == next->index) {
                    found = true;

                    //Verify that the switch matches
                    int rr_iswitch = rr_graph.edge_switch(RRNodeId(trace->index), iedge);
                    if (trace->iswitch != rr_iswitch) {
                        VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Traceback mismatched switch type: traceback %d rr_graph %d (RR nodes %d -> %d)\n",
                                        trace->iswitch, rr_iswitch,
                                        trace->index, to_node);
                    }
                    break;
                }
            }

            if (!found) {
                VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Traceback no RR edge between RR nodes %d -> %d\n", trace->index, next->index);
            }

            //Recurse
            return validate_traceback_recurr(next, seen_rr_nodes);
        }
    }

    VTR_ASSERT(!next);
    return true; //End of traceback
}

t_trace*
alloc_trace_data() {
    return (t_trace*)malloc(sizeof(t_trace));
}

void free_trace_data(t_trace* tptr) {
    free(tptr);
}

void free_traceback(t_trace* tptr) {
    while (tptr != nullptr) {
        t_trace* tempptr = tptr->next;
        free_trace_data(tptr);
        tptr = tempptr;
    }
}
