#pragma once

#include <memory>
#include "BlobDetector.h"

struct Run;

class Node{
    public:
      Run *data;
      int rank;
      Node* parent;
      Node(Run *t) : data(t), rank(0), parent(this) { }

//  template <class T>
  Node* find();
//  template <class T>
  void merge(Node* b);
};
