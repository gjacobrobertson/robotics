#include <common/DisjointSet.h>

template <class T>
DisjointSet::Node<T> DisjointSet::find(DisjointSet::Node<T> *node) {
  if (node != node->parent) {
    node->parent = find(node->parent);
  }
  return node->parent;
}

template <class T>
void DisjointSet::merge(DisjointSet::Node<T> *a, DisjointSet::Node<T> *b) {
  a = find(a);
  b = find(b);
  if (a == b) {
    return;
  }
  if (a->rank < b->rank) {
    a->parent = b;
  } else if (a->rank > b->rank) {
    b->parent = a;
  } else {
    b->parent = a;
    a->rank++;
  }
}

