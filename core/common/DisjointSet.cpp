#include <common/DisjointSet.h>

template <class T>
std::shared_ptr<DisjointSet::Node<T>> DisjointSet::find(std::shared_ptr<DisjointSet::Node<T>> node) {
  if (node != node->parent) {
    node->parent = find(node->parent);
  }
  return node->parent;
}

template <class T>
void DisjointSet::merge(std::shared_ptr<DisjointSet::Node<T>> a, std::shared_ptr<DisjointSet::Node<T>> b) {
  a = find(a);
  b = find(b);
  if (a->rank < b->rank) {
    a->parent = b;
  } else if (a->rank > b->rank) {
    b->parent = a;
  } else {
    b->parent = a;
    a->rank++;
  }
}

