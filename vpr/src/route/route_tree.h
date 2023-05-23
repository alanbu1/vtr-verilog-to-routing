#pragma once

#include <functional>
#include <iostream>
#include <iterator>
#include <list>
#include <stack>
#include <unordered_map>
#include <unordered_set>

#include "connection_based_routing_fwd.h"
#include "route_tree_fwd.h"
#include "rr_graph_fwd.h"
#include "spatial_route_tree_lookup.h"
#include "vtr_optional.h"

class RTRecIterator;

/**
 * @brief A single route tree node
 *
 * Structure describing one node in a RouteTree. */
class RouteTreeNode {
    friend class RouteTree;

  public:
    RouteTreeNode() = delete;
    RouteTreeNode(const RouteTreeNode&);
    RouteTreeNode(RouteTreeNode&&);
    RouteTreeNode& operator=(const RouteTreeNode&);
    RouteTreeNode& operator=(RouteTreeNode&&);

    RouteTreeNode(RRNodeId inode, RRSwitchId parent_switch, vtr::optional<RouteTreeNode&> parent);

    /** ID of the rr_node that corresponds to this rt_node. */
    RRNodeId inode;
    /** Switch type driving this node (by its parent). */
    RRSwitchId parent_switch;
    /** Reference to this node's parent */
    vtr::optional<RouteTreeNode&> parent;
    /** Should this node be put on the heap as part of the partial
     * routing to act as a source for subsequent connections? */
    bool re_expand;
    /** Net pin index associated with the rt_node. This value
     * ranges from 1 to fanout [1..num_pins-1]. For cases when
     * different speed paths are taken to the same SINK for
     * different pins, inode cannot uniquely identify each SINK,
     * so the net pin index guarantees an unique identification
     * for each SINK rt_node. For non-SINK nodes and for SINK
     * nodes with no associated net pin index, (i.e. special 
     * SINKs like the source of a clock tree which do not
     * correspond to an actual netlist connection), the value
     * for this member should be set to OPEN (-1). */
    int net_pin_index;
    /** Total downstream capacitance from this rt_node. That is,
     * the total C of the subtree rooted at the current node,
     * including the C of the current node. */
    float C_downstream;
    /** Total upstream resistance from this rt_node to the net
     * source, including any device_ctx.rr_nodes[].R of this node. */
    float R_upstream;
    /** Time delay for the signal to get from the net source to this node.
     * Includes the time to go through this node. */
    float Tdel;

    /** Copy child to the front of _child_nodes.
     * Returns a reference to the added node. */
    RouteTreeNode& add_child(const RouteTreeNode&);

    /** Emplace child to the front of _child_nodes.
     * For best performance, call with constructor args
     * (will construct the node in the parent's list directly and save a copy) */
    template<class... Args>
    RouteTreeNode& emplace_child(Args&&... args);

    /** Remove child node by value. O(N) operation. */
    void remove_child(const RouteTreeNode&);

    /** Remove child node by list iterator state. O(1) operation. */
    void remove_child(std::list<RouteTreeNode>::iterator&);

    /** Iterate through child nodes and remove if p returns true.
     * Best way to use is through a lambda which takes "child_node" as an argument. */
    void remove_child_if(const std::function<bool(RouteTreeNode&)>&);

    /** Get a list of child nodes. Useful for traversal.
     * Adding or removing child nodes manually can result in bugs. */
    std::list<RouteTreeNode>& child_nodes(void) const;

    /** Print information about this subtree to stdout. */
    void print(void) const;

    /** Equality operator. For now, just compare the addresses */
    friend bool operator==(const RouteTreeNode& lhs, const RouteTreeNode& rhs) {
        return &lhs == &rhs;
    }
    friend bool operator!=(const RouteTreeNode& lhs, const RouteTreeNode& rhs) {
        return !(lhs == rhs);
    }

  private:
    void update_parent_links(void);
    void print_x(int depth) const;

    /** Container for child nodes. Every node "owns" the memory
     * for their child nodes -> no explicit destructor is needed. */
    std::list<RouteTreeNode> _child_nodes;
};

/** Top level route tree used in timing analysis and keeping partial routing state.
 * Contains the root node and a lookup from RRNodeIds to RouteTreeNode&s in the tree. */
class RouteTree {
  public:
    RouteTree() = delete;
    RouteTree(const RouteTree&);
    RouteTree(RouteTree&&);
    RouteTree& operator=(const RouteTree&);
    RouteTree& operator=(RouteTree&&);

    /** Return a RouteTree initialized to inode */
    RouteTree(RRNodeId inode);
    /** Return a RouteTree initialized to the source of nets[inet] */
    RouteTree(ParentNetId inet);
    /** Move existing RouteTreeNode into a RouteTree (i.e. built outside this class) */
    RouteTree(RouteTreeNode&& root);

    /** Add the most recently finished wire segment to the routing tree, and
     * update the Tdel, etc. numbers for the rest of the routing tree. hptr
     * is the heap pointer of the SINK that was reached, and target_net_pin_index
     * is the net pin index corresponding to the SINK that was reached. This routine
     * returns a tuple: RouteTreeNode of the branch it adds to the route tree and
     * RouteTreeNode of the SINK it adds to the routing. */
    std::tuple<vtr::optional<RouteTreeNode&>, vtr::optional<RouteTreeNode&>>
    update_from_heap(t_heap* hptr, int target_net_pin_index, SpatialRouteTreeLookup* spatial_rt_lookup, bool is_flat);

    /** Reload timing values (R_upstream, C_downstream, Tdel).
     * Can take a RouteTreeNode& to do an incremental update.
     * Note that update_from_heap already does this. */
    void reload_timing(vtr::optional<RouteTreeNode&> from_node = vtr::nullopt);

    /** Get the RouteTreeNode corresponding to the RRNodeId. Returns nullopt if not found */
    vtr::optional<RouteTreeNode&> find_by_rr_id(RRNodeId rr_node) const;

    /** Check the consistency of this route tree. Looks for:
     * - invalid parent-child links
     * - invalid timing values
     * - congested SINKs
     * Returns true if OK. */
    bool is_valid(void) const;

    /** Check if the tree has any overused nodes (-> the tree is congested).
     * Returns true if not congested */
    bool is_uncongested(void) const;

    /** Print information about this route tree to stdout. */
    void print(void) const;

    /** Prune overused nodes from the tree.
     * Also prune unused non-configurable nodes if non_config_node_set_usage is provided (see get_non_config_node_set_usage)
     * Returns nullopt if the entire tree is pruned. */
    vtr::optional<RouteTree&> prune(CBRR& connections_inf, std::vector<int>* non_config_node_set_usage = nullptr);

    /** Remove all sinks and mark the remaining nodes as un-expandable.
     * This is used after routing a clock net.
     * TODO: is this function doing anything? Try running without it */
    void freeze(void);

    /** Count configurable edges to non-configurable node sets. (rr_nonconf_node_sets index -> int)
     * Required when using prune() to remove non-configurable nodes. */
    std::vector<int> get_non_config_node_set_usage(void) const;

    /** Wrapper for the recursive iterator.
     * Only for syntax purposes: for(x: tree.all_nodes()) will be more readable than for(x: tree).
     * This is Java-ish and I'm not sure about the performance impact (why create an object to iterate?) but let's see. */
    using iterator = RTRecIterator;
    class Iterable {
      public:
        Iterable(const RouteTreeNode& root)
            : _root(root) {}
        const RouteTreeNode& _root;
        iterator begin() const;
        iterator end() const;
    };

    /** Get an iterable for all nodes under this RouteTree (walks the tree).
     * Take care to iterate by reference.
     * Copying a RouteTreeNode is a recursive action and it zeroes out the parent reference. */
    Iterable all_nodes(void) const;

    /** Get a reference to the root RouteTreeNode. */
    inline const RouteTreeNode& root(void) const { return _root; }

  private:
    void update_references(RouteTreeNode& rt_node);

    std::tuple<vtr::optional<RouteTreeNode&>, vtr::optional<RouteTreeNode&>>
    add_subtree_from_heap(t_heap* hptr, int target_net_pin_index, bool is_flat);

    void add_non_configurable_nodes(RouteTreeNode& rt_node,
                                    bool reached_by_non_configurable_edge,
                                    std::unordered_set<RRNodeId>& visited,
                                    bool is_flat);

    void load_new_subtree_R_upstream(RouteTreeNode& from_node);
    float load_new_subtree_C_downstream(RouteTreeNode& from_node);
    RouteTreeNode& update_unbuffered_ancestors_C_downstream(RouteTreeNode& from_node);
    void load_route_tree_Tdel(RouteTreeNode& from_node, float Tarrival);

    bool is_valid_x(const RouteTreeNode& rt_node) const;
    bool is_uncongested_x(const RouteTreeNode& rt_node) const;

    vtr::optional<RouteTreeNode&>
    prune_x(RouteTreeNode& rt_node,
            CBRR& connections_inf,
            bool force_prune,
            std::vector<int>* non_config_node_set_usage);

    void freeze_x(RouteTreeNode& rt_node);

    /** Root node. */
    RouteTreeNode _root;

    /** Lookup from RRNodeIds to RouteTreeNodes in the tree.
     * Note that calling node.add_child or node.emplace_child outside of the tree
     * will not update this lookup. Try to implement your operation inside RouteTree
     * rather than editing RouteTreeNodes.
     * In some cases the same SINK node is put into the tree multiple times in a
     * single route. To model this, we are putting in separate rt_nodes in the route
     * tree if we go to the same SINK more than once. rr_node_to_rt_node[inode] will
     * therefore store the last rt_node created of all the SINK nodes with the same
     * index "inode". */
    std::unordered_map<RRNodeId, vtr::optional<RouteTreeNode&>> _rr_node_to_rt_node;
};

/* Recursive iterator implementation for a RouteTreeNode.
 * This replaces the traceback traversal, which goes over the tree
 * in a depth-first, pre-order fashion.
 * This implementation uses a stack to do this,
 * which is not optimal for copying, so expect bad performance if using
 * <algorithm> fns on it.
 * Ideas about how to do it with less state are welcome. */
class RTRecIterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = RouteTreeNode;
    using pointer = RouteTreeNode*;
    using reference = const RouteTreeNode&;

    RTRecIterator() = default;
    RTRecIterator(vtr::optional<const RouteTreeNode&> node);

    /* Required operators for a forward iterator. */
    reference operator*() const;
    RTRecIterator& operator++();
    RTRecIterator operator++(int);
    bool operator==(const RTRecIterator& rhs);
    bool operator!=(const RTRecIterator& rhs);

  private:
    /* Stack of nodes to visit. */
    std::stack<vtr::optional<const RouteTreeNode&>> _stack;
    /* The root node of the iterator. Useful for comparisons. */
    vtr::optional<const RouteTreeNode&> _root;
};

/* Emplace child to the front of _child_nodes.
 * For best performance, call with constructor args
 * (will construct the node in the parent's list directly and save a copy)
 * Implemented in this file to enable template deduction */
template<class... Args>
RouteTreeNode& RouteTreeNode::emplace_child(Args&&... args) {
    _child_nodes.emplace_front(std::forward<Args>(args)...);
    RouteTreeNode& new_node = _child_nodes.front();
    new_node.parent = *this; // Zeroed out after copy constructor
    return new_node;
}
