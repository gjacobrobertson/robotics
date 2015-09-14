#include "Node.h"

//template <class T>
Node* Node::find() {
  if (this != parent) {
    parent = parent->find();
  }
  //else {
  //  if (depth > 1) cout << "Depth: " << depth << endl;
//  }
  return parent;
}

//template <class T>
void Node::merge(Node *b) {
  find();
  b -> find();
  if (parent == b->parent) { // Both already in same blob
   // if (data->end > b->parent->data->xf)
  //    b->parent->data->xf = data->end;
//    b->parent->data->color_ct += data->color_ct;
    return;
  }
  if (rank < b->rank) { // Update b
    if (parent->data->xf > b->parent->data->xf)
      b->parent->data->xf = parent->data->xf;
    if (parent->data->xi < b->parent->data->xi)
      b->parent->data->xi = parent->data->xi;
    if (parent->data->yf > b->parent->data->yf)
      b->parent->data->yf = parent->data->yf;
    if (parent->data->yi < b->parent->data->yi)
      b->parent->data->yi = parent->data->yi;
    b->parent->data->color_ct += parent->data->color_ct;
    parent = b;
  } else if (rank > b->rank) { // update this
    if (parent->data->xf < b->parent->data->xf)
      parent->data->xf = b->parent->data->xf;
    if (parent->data->xi > b->parent->data->xi)
      parent->data->xi = b->parent->data->xi;
    if (b->parent->data->yf > parent->data->yf)
      parent->data->yf = b->parent->data->yf;
    if (parent->data->yi > b->parent->data->yi)
      parent->data->yi = b->parent->data->yi;
    parent->data->color_ct += b->parent->data->color_ct;
    b->parent = this;
  } else { // update this
    if (parent->data->xf < b->parent->data->xf)
      parent->data->xf = b->parent->data->xf;
    if (parent->data->xi > b->parent->data->xi)
      parent->data->xi = b->parent->data->xi;
    if (b->parent->data->yf > parent->data->yf)
      parent->data->yf = b->parent->data->yf;
    if (parent->data->yi > b->parent->data->yi)
      parent->data->yi = b->parent->data->yi;
    parent->data->color_ct += b->parent->data->color_ct;
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
