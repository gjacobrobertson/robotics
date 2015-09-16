#include <vision/structures/Blob.h>
#include <iostream>

bool sortBlobAreaPredicate(Blob* left, Blob* right) {
  return left->dx * left->dy > right->dx * right->dy;
}
