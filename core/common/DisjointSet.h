#pragma once

#include <memory>

namespace DisjointSet {
  template <class T>
  class Node{
    public:
      T data;
      int rank;
      Node<T>* parent;
      Node<T>(T t) : data(t), rank(0), parent(this) { }
  };
  template <class T>
  Node<T>* find(Node<T>*);
  template <class T>
  void merge(Node<T>*, Node<T>*);
}
