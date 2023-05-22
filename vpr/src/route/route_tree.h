#pragma once

#include <functional>
#include <iostream>
#include <iterator>
#include <list>
#include <stack>
#include <unordered_map>
#include <unordered_set>

#include "rr_graph_fwd.h"
#include "vtr_optional.h"

class RouteTree;
class RTIterator;

/**
 * @brief A single route tree node
 *
 * Structure describing one node in a routing tree (used to get net delays
 * incrementally during routing, as pieces are being added).
 *
 * parent: Reference to this node's parent
 * children: Pointer to a linked list of linked_rt_edge.  Each one of
 *                the linked list entries gives a child of this node.
 * re_expand: Should this node be put on the heap as part of the partial
 *             routing to act as a source for subsequent connections?
 * parent_switch: Index of the switch type driving this node (by its
 *                 parent).
 * inode: index (ID) of the rr_node that corresponds to this rt_node.
 * net_pin_index: Net pin index associated with the rt_node. This value
 *                 ranges from 1 to fanout [1..num_pins-1]. For cases when
 *                 different speed paths are taken to the same SINK for
 *                 different pins, inode cannot uniquely identify each SINK,
 *                 so the net pin index guarantees an unique identification
 *                 for each SINK rt_node. For non-SINK nodes and for SINK
 *                 nodes with no associated net pin index, (i.e. special
 *                 SINKs like the source of a clock tree which do not
 *                 correspond to an actual netlist connection), the value
 *                 for this member should be set to OPEN (-1).
 * C_downstream: Total downstream capacitance from this rt_node.  That is,
 *                the total C of the subtree rooted at the current node,
 *                including the C of the current node.
 * R_upstream: Total upstream resistance from this rt_node to the net
 *              source, including any device_ctx.rr_nodes[].R of this node.
 * Tdel: Time delay for the signal to get from the net source to this node.
 *        Includes the time to go through this node.
 */
class RouteTreeNode {
  public:
    RouteTreeNode() = delete;
    RouteTreeNode(const RouteTreeNode&);
    RouteTreeNode(RouteTreeNode&&);
    RouteTreeNode& operator=(const RouteTreeNode&);
    RouteTreeNode& operator=(RouteTreeNode&&);

    RouteTreeNode(RRNodeId inode, RRSwitchId parent_switch, vtr::optional<RouteTreeNode&> parent, vtr::optional<RouteTree&> tree);

    RRNodeId inode;
    RRSwitchId parent_switch;
    vtr::optional<RouteTreeNode&> parent;
    vtr::optional<RouteTree&> tree;
    bool re_expand;
    int net_pin_index;
    float C_downstream;
    float R_upstream;
    float Tdel;
    std::list<RouteTreeNode> _child_nodes;

    /* Copy child to the back of _child_nodes.
     * Returns a reference to the added node. */
    RouteTreeNode& add_child(const RouteTreeNode&);

    /* Copy child to the front of _child_nodes.
     * Returns a reference to the added node. */
    RouteTreeNode& add_child_front(const RouteTreeNode&);

    /* Emplace child to the back of _child_nodes.
     * For best performance, call with constructor args
     * (will construct the node in the parent's list directly and save a copy) */
    template<class... Args>
    RouteTreeNode& emplace_child(Args&&... args);

    /* Emplace child to the front of _child_nodes.
     * For best performance, call with constructor args
     * (will construct the node in the parent's list directly and save a copy) */
    template<class... Args>
    RouteTreeNode& emplace_child_front(Args&&... args);

    /* Remove child node by value. O(N) operation. */
    void remove_child(const RouteTreeNode&);

    /* Remove child node by list iterator state. O(1) operation. */
    void remove_child(std::list<RouteTreeNode>::iterator&);

    /* Iterate through child nodes and remove if p returns true.
     * Best way to use is through a lambda which takes "child_node" as an argument. */
    void remove_child_if(const std::function<bool(RouteTreeNode&)>&);

    /* Get a list of child nodes. Useful for traversal.
     * Adding or removing child nodes manually can result in bugs. */
    std::list<RouteTreeNode>& child_nodes(void) const;

    /* For now, just compare their addresses */
    friend bool operator==(const RouteTreeNode& lhs, const RouteTreeNode& rhs) {
        return &lhs == &rhs;
    }
    friend bool operator!=(const RouteTreeNode& lhs, const RouteTreeNode& rhs) {
        return !(lhs == rhs);
    }

  private:
    void update_parent_links(void);
};

/* Contains the root node and a lookup from RRNodeIds to RouteTreeNode&s in the tree.
 * Previous comment about rr_node_to_rt_node:
 * ---
 * In some cases the same SINK node is put into the tree multiple times in a
 * single route. To model this, we are putting in separate rt_nodes in the route
 * tree if we go to the same SINK more than once. rr_node_to_rt_node[inode] will
 * therefore store the last rt_node created of all the SINK nodes with the same
 * index "inode". This is okay because the mapping is only used in this file to
 * quickly figure out where rt_nodes that we are branching off of (for nets with
 * fanout > 1) are, and we will never branch off a SINK. */
class RouteTree {
  public:
    RouteTree() = delete;
    RouteTree(const RouteTree&);
    RouteTree(RouteTree&&);
    RouteTree& operator=(const RouteTree&);
    RouteTree& operator=(RouteTree&&);

    RouteTree(RRNodeId inode);

    /* API to iterate over all nodes in this RouteTree.
     * Take care to iterate by reference.
     * Copying RouteTreeNodes will yield unexpected results. */
    using iterator = RTIterator;
    iterator begin() const;
    iterator end() const;

    RouteTreeNode root;
    std::unordered_map<RRNodeId, vtr::optional<RouteTreeNode&>> rr_node_to_rt_node;

  private:
    void update_references(RouteTreeNode& rt_node);
};

/* Iterator implementation for a RouteTreeNode.
 * This replaces the traceback traversal, which goes over the tree
 * in a depth-first, pre-order fashion.
 * This implementation uses a stack to do this,
 * which is not optimal for copying, so expect bad performance if using
 * <algorithm> fns on it.
 * Ideas about how to do it with less state are welcome. */
class RTIterator {
  public:
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = RouteTreeNode;
    using pointer = RouteTreeNode*;
    using reference = const RouteTreeNode&;

    RTIterator() = default;
    RTIterator(vtr::optional<const RouteTreeNode&> node);

    /* Required operators for a forward iterator. */
    reference operator*() const;
    RTIterator& operator++();
    RTIterator operator++(int);
    bool operator==(const RTIterator& rhs);
    bool operator!=(const RTIterator& rhs);

  private:
    /* Stack of nodes to visit. */
    std::stack<vtr::optional<const RouteTreeNode&>> _stack;
    /* The root node of the iterator. Useful for comparisons. */
    vtr::optional<const RouteTreeNode&> _root;
};

/* Emplace child to the back of _child_nodes.
 * For best performance, call with constructor args
 * (will construct the node in the parent's list directly and save a copy)
 * Implemented in this file to enable template deduction */
template<class... Args>
RouteTreeNode& RouteTreeNode::emplace_child(Args&&... args) {
    _child_nodes.emplace_back(std::forward<Args>(args)...);
    RouteTreeNode& new_node = _child_nodes.back();
    if (tree)
        tree.value().rr_node_to_rt_node[new_node.inode] = new_node;
    new_node.parent = *this; // Zeroed out after copy constructor
    new_node.tree = tree;
    return new_node;
}

/* Emplace child to the back of _child_nodes.
 * For best performance, call with constructor args
 * (will construct the node in the parent's list directly and save a copy)
 * Implemented in this file to enable template deduction */
template<class... Args>
RouteTreeNode& RouteTreeNode::emplace_child_front(Args&&... args) {
    _child_nodes.emplace_front(std::forward<Args>(args)...);
    RouteTreeNode& new_node = _child_nodes.front();
    if (tree)
        tree.value().rr_node_to_rt_node[new_node.inode] = new_node;
    new_node.parent = *this; // Zeroed out after copy constructor
    new_node.tree = tree;
    return new_node;
}
