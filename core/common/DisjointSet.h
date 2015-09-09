#pragma once

#include <memory>

namespace DisjointSet {
  template <class T>
  class Node: std::enable_shared_from_this<Node<T>> {
    public:
      T data;
      int rank;
      std::shared_ptr<Node<T>> parent;
      Node<T>(T t) : data(t), rank(0), parent(this->shared_from_this()) { }
  };
  template <class T>
  std::shared_ptr<Node<T>> find(std::shared_ptr<Node<T>>);
  template <class T>
  void merge(std::shared_ptr<Node<T>>, std::shared_ptr<Node<T>>);
}
