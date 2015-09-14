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
      ~Node();

      Node* find();
      void merge(Node* b);
      void mergeStats(Node* b);
      void print();
};
