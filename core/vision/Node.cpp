#include "Node.h"

Node* Node::find() {
  if (this != parent) {
    parent = parent->find();
  }
  return parent;
}

void Node::merge(Node *b) {
  Node *a = find();
  b = b -> find();
  if (a == b) { // Both already in same blob
    return;
  }
  if (a->rank < b->rank) {
    a->parent = b;
    b->mergeStats(a);
  } else if (a->rank > b->rank) {
    b->parent = a;
    a->mergeStats(b); 
  } else {
    b->parent = a;
    a->rank++;
    a->mergeStats(b);
  }
}

//merge stats from Node *b into this
void Node::mergeStats(Node *b)
{
  data->xi = min(data->xi, b->data->xi);
  data->xf = max(data->xf, b->data->xf);
  data->yi = min(data->yi, b->data->yi);
  data->yf = max(data->yf, b->data->yf);
  data->color_ct += b->data->color_ct;
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
