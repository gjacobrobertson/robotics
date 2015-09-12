#include "Node.h"

//template <class T>
Node* Node::find() {
  if (this != parent) {
    parent = parent->find();
  }
  return parent;
}

//template <class T>
void Node::merge(Node *b) {
  find();
  b -> find();
  if (parent == b->parent) {
    return;
  }
  if (rank < b->rank) {
    parent = b;
  } else if (rank > b->rank) {
    b->parent = this;
  } else {
    b->parent = this;
    rank++;
  }
}

