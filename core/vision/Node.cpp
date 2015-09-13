#include "Node.h"

//template <class T>
Node* Node::find(int depth) {
  if (this != parent) {
    parent = parent->find(depth + 1);
  }
  else {
    if (depth > 1) cout << "Depth: " << depth << endl;
  }
  return parent;
}

//template <class T>
void Node::merge(Node *b) {
  find(0);
  b -> find(0);
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

Node::~Node()
{
  delete data;
}

void Node::print()
{
  cout << "Run(" << (int)data->color << ") ("<<data->color_ct<<")";
  cout << " (" <<data->xi << "," << data->yi << ") to (" << data->xf << "," << data->yf << ")";
  cout << " Parent: ";
  if (parent != this)
    parent->print();
  else
    cout << "none";
  cout << endl;
}
