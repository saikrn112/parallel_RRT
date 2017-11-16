#ifndef NODE_HPP
#define NODE_HPP

#include <type_traits>
#include <vector>

namespace RRT {

/**
 * Base class for an RRT tree node.
 *
 * Vector2d The datatype representing the state in the space the RRT
 * will be searching.
 */

class Node {
private:
  std::vector<double> _vec;
  Vector2d _state;
  std::list<Node*> _children;
  Node* _parent;

public:
  Node(const Vector2d& , Node*, int);
  const Node* parent() const;
  int depth() const;
  const Vector2d& state() const;
  std::vector<double>* coordinates();

}

Node::Node(const Vector2d& state, Node* parent = nullptr, int dimensions = 2):
_parent(parent),
_state(state),
_vec(dimensions) {
  if (_parent) {
      _parent->_children.push_back(this);
  }
  for (int i = 0; i < dimensions; i++) {
      _vec[i] = state[i];
  }
}

const Node* Node::parent() const {
  return _parent;
}

/**
 * Gets the number of ancestors (parent, parent's parent, etc) that
 * the node has.
 * Returns 0 if it doesn't have a parent.
 */
int Node::depth() const {
    int n = 0;
    for (Node* ancestor = _parent; ancestor != nullptr;
         ancestor = ancestor->_parent) {
        n++;
    }
    return n;
}

/**
 * The @state property is the point in the state-space that this
 * Node represents.  Generally this is a vector (could be 2d, 3d, etc)
 */
const Vector2d& state() const {
  return _state;
}

std::vector<double>* coordinates() {
  return &_vec;
}

} // namespace RRT
