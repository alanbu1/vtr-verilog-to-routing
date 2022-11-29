#include "route_tree_type.h"
#include "globals.h"

/* Construct a new RouteTreeNode.
 * Doesn't add the node to parent's child_nodes! (see add_child) */
RouteTreeNode::RouteTreeNode(RRNodeId _inode, RRSwitchId _parent_switch, vtr::optional<RouteTreeNode&> _parent, vtr::optional<RouteTree&> _root)
    : inode(_inode)
    , parent_switch(_parent_switch)
    , parent(_parent)
    , root(_root) {
    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    re_expand = true;
    net_pin_index = OPEN;
    C_downstream = rr_graph.node_C(_inode);
    R_upstream = rr_graph.node_R(_inode);
    Tdel = 0.5 * R_upstream * C_downstream;
}

/* Copy & move: update parent links when things are moved.
 * Note that the parent of the copy/moved node will be set to nullopt.
 * The rest of the tree will have consistent links.
 * I didn't use copy-and-swap here. Maybe this saves performance? */
RouteTreeNode::RouteTreeNode(const RouteTreeNode& rhs)
    : RouteTreeNode(rhs.inode, rhs.parent_switch, vtr::nullopt, vtr::nullopt) {
    re_expand = rhs.re_expand;
    net_pin_index = rhs.net_pin_index;
    C_downstream = rhs.C_downstream;
    R_upstream = rhs.R_upstream;
    Tdel = rhs.Tdel;
    _child_nodes = rhs._child_nodes;
    update_parent_links();
}

RouteTreeNode::RouteTreeNode(RouteTreeNode&& rhs)
    : RouteTreeNode(rhs.inode, rhs.parent_switch, vtr::nullopt, vtr::nullopt) {
    re_expand = rhs.re_expand;
    net_pin_index = rhs.net_pin_index;
    C_downstream = rhs.C_downstream;
    R_upstream = rhs.R_upstream;
    Tdel = rhs.Tdel;
    _child_nodes = std::move(rhs._child_nodes);
    update_parent_links();
}

RouteTreeNode& RouteTreeNode::operator=(const RouteTreeNode& rhs) {
    inode = rhs.inode;
    parent_switch = rhs.parent_switch;
    re_expand = rhs.re_expand;
    net_pin_index = rhs.net_pin_index;
    C_downstream = rhs.C_downstream;
    R_upstream = rhs.R_upstream;
    Tdel = rhs.Tdel;
    _child_nodes = rhs._child_nodes;
    root = vtr::nullopt;
    parent = vtr::nullopt;
    update_parent_links();
    return *this;
}

RouteTreeNode& RouteTreeNode::operator=(RouteTreeNode&& rhs) {
    inode = rhs.inode;
    parent_switch = rhs.parent_switch;
    re_expand = rhs.re_expand;
    net_pin_index = rhs.net_pin_index;
    C_downstream = rhs.C_downstream;
    R_upstream = rhs.R_upstream;
    Tdel = rhs.Tdel;
    _child_nodes = std::move(rhs._child_nodes);
    root = vtr::nullopt;
    parent = vtr::nullopt;
    update_parent_links();
    return *this;
}

void RouteTreeNode::update_parent_links(void) {
    for (auto& child : _child_nodes) {
        child.parent = *this;
    }
}

/* Copy child to the back of _child_nodes.
 * Returns a reference to the added node. */
RouteTreeNode& RouteTreeNode::add_child(const RouteTreeNode& x) {
    _child_nodes.push_back(x);
    RouteTreeNode& new_node = _child_nodes.back();
    if (root)
        root.value().rr_node_to_rt_node[x.inode] = new_node;
    new_node.parent = *this; // Zeroed out after copy constructor
    new_node.root = root;
    return new_node;
}

/* Copy child to the front of _child_nodes.
 * Returns a reference to the added node. */
RouteTreeNode& RouteTreeNode::add_child_front(const RouteTreeNode& x) {
    _child_nodes.push_front(x);
    RouteTreeNode& new_node = _child_nodes.front();
    if (root)
        root.value().rr_node_to_rt_node[x.inode] = new_node;
    new_node.parent = *this; // Zeroed out after copy constructor
    new_node.root = root;
    return new_node;
}

/* Remove child node by value. O(N) operation. */
void RouteTreeNode::remove_child(const RouteTreeNode& x) {
    if (root)
        root.value().rr_node_to_rt_node.erase(x.inode);
    _child_nodes.remove(x);
}

/* Remove child node by list iterator state. O(1) operation.
 * If in a for loop, take care about the iterator state:
 * https://stackoverflow.com/questions/596162 */
void RouteTreeNode::remove_child(std::list<RouteTreeNode>::iterator& it) {
    if (root)
        root.value().rr_node_to_rt_node.erase(it->inode);
    it = _child_nodes.erase(it);
}

/* Iterate through child nodes and remove if p returns true.
 * Also see: https://en.cppreference.com/w/cpp/container/list/remove */
void RouteTreeNode::remove_child_if(const std::function<bool(RouteTreeNode&)>& p) {
    _child_nodes.remove_if([&](auto& child) {
        if (p(child)) {
            if (root) {
                root.value().rr_node_to_rt_node.erase(child.inode);
            }
            return true;
        } else {
            return false;
        }
    });
}

/* Get a list of child nodes. Useful for traversal.
 * Adding or removing child nodes manually can result in bugs. */
std::list<RouteTreeNode>& RouteTreeNode::child_nodes(void) const {
    /* If we don't cast this, gcc will complain because we are returning
     * a non-const reference from a const function. That doesn't make sense in my
     * opinion: this getter doesn't modify anything by itself, so it should
     * still be const */
    return const_cast<std::list<RouteTreeNode>&>(_child_nodes);
}

/* Construct a top-level route tree. */
RouteTree::RouteTree(RRNodeId _inode)
    : root(RouteTreeNode(_inode, RRSwitchId(OPEN), vtr::nullopt, *this)) {
    rr_node_to_rt_node[_inode] = root;
}

/* Copy & move: update references in rr_node_to_rt_node and update root pointers. */
RouteTree::RouteTree(const RouteTree& rhs)
    : root(rhs.root) {
    update_references(root);
}

RouteTree::RouteTree(RouteTree&& rhs)
    : root(std::move(rhs.root)) {
    update_references(root);
}

RouteTree& RouteTree::operator=(const RouteTree& rhs) {
    root = rhs.root;
    rr_node_to_rt_node.clear();
    update_references(root);
    return *this;
}

RouteTree& RouteTree::operator=(RouteTree&& rhs) {
    root = std::move(rhs.root);
    rr_node_to_rt_node.clear();
    update_references(root);
    return *this;
}

RouteTree::iterator RouteTree::begin(void) const {
    return iterator(root);
}

RouteTree::iterator RouteTree::end(void) const {
    return iterator();
}

/* Walk the tree, re-populate rr_node_to_rt_node and root. */
void RouteTree::update_references(RouteTreeNode& rt_node) {
    rt_node.root = *this;
    rr_node_to_rt_node[rt_node.inode] = rt_node;
    for (auto& child : rt_node.child_nodes()) {
        update_references(child);
    }
}

/* Iterator implementation for a RouteTreeNode.
 * This replaces the traceback traversal, which goes over the tree
 * in a depth-first, pre-order fashion. */
RTIterator::RTIterator(vtr::optional<const RouteTreeNode&> node) {
    if (!node) return;
    _root = node;
    _stack.push(node);
}

/* UB if stack is empty. (iterator == end()) */
RTIterator::reference RTIterator::operator*() const {
    return _stack.top().value();
}

/* Move to the next node. */
RTIterator& RTIterator::operator++() {
    if (_stack.empty()) // we are at end(), do nothing
        return *this;
    const RouteTreeNode& node = _stack.top().value();
    _stack.pop();
    if (node.child_nodes().empty()) // no child nodes to expand, do nothing
        return *this;
    // push child nodes in reverse order
    for (auto it = node.child_nodes().rbegin(); it != node.child_nodes().rend(); ++it) {
        _stack.push(*it);
    }
    return *this;
}

/* Make a copy of this iterator and move to the next node. (expect performance hit) */
RTIterator RTIterator::operator++(int) {
    RTIterator tmp = *this;
    ++(*this);
    return tmp;
}

/* Compare the original root and the current stack top. */
bool RTIterator::operator==(const RTIterator& rhs) {
    if (_stack.empty() && rhs._stack.empty()) // both are end()
        return true;
    if (_stack.empty() || rhs._stack.empty()) // only one of the stacks are empty
        return false;
    if (_root != rhs._root) // both stacks full, but different root nodes
        return false;
    // true if same root nodes and same stack tops
    // (a bug if the tree changed in between but that's going to break things anyway)
    return *(*this) == *(rhs);
}

bool RTIterator::operator!=(const RTIterator& rhs) {
    return !(*this == rhs);
}
