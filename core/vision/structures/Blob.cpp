#include <vision/structures/Blob.h>
#include <iostream>

bool sortBlobAreaPredicate(Blob* left, Blob* right) {
  return left->dx * left->dy < right->dx * right->dy;
}

void Blob::print(){

  std::cout << "Blob("<<(int)color<<") at (" << xi <<","<<yi<<") to ("<<xf <<","<<yf<<") width = " << dx << " height = " << dy << " " <<avgWidth <<std::endl;

}
