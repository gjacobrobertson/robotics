#include "BlobDetector.h"
#include "Node.h"
#include "structures/Blob.h"

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
  dx = 4;
  dy = 2;
//  width = 5;
//  height = 5;
}

vector<Blob*> BlobDetector::findBlobs(unsigned char* segImg) {
//  if(camera_ == Camera::BOTTOM) return;
//  cout << "Find Blobs" << endl;
  vector<vector<Node*>> rows = findRuns(segImg);
  
  mergeRuns(rows);

  vector<Blob*> blob_list;
  Blob *blob;

  for (int i=0; i<rows.size(); i++){
    for (int j=0; j<rows[i].size(); j++) {
      if (rows[i][j]->data->color == c_ORANGE){
//        cout <<"Row: " << i << " ";
//        rows[i][j]->print();
      }
      if (rows[i][j]->parent == rows[i][j]){  // Points to self
        blob = new Blob();
        blob->xi = rows[i][j]->data->xi;
        blob->xf = rows[i][j]->data->xf;
        blob->yi = rows[i][j]->data->yi;
        blob->yf = rows[i][j]->data->yf;
        blob->dx = (blob->xf - blob->xi) + 1;
        blob->dy = (blob->yf - blob->yi) + 1;
        blob->color = rows[i][j]->data->color;
        blob->avgWidth = (rows[i][j]->data->color_ct) * (dy*1.0) / blob->dy;
        blob_list.push_back(blob);
//        cout << "BlobDetet: " << (int)blob->color << endl;
      }
    }
  }

//  for (int i=0; i<blob_list.size(); i++)
//    blob_list[i]->print();

  for (int i=0; i < rows.size(); i++) {
    for (int j=0; j < rows[i].size(); j++){
      delete rows[i][j];
    }
    rows[i].clear();
  }
  rows.clear();
//  for (int i=0; i<blob_list.size(); i++)
//    delete blob_list[i];
  return blob_list;
}

vector<vector<Node*>> BlobDetector::findRuns(unsigned char* segImg) {
  vector<vector<Node*>> rows;
  //unsigned char* segImg = vblocks_.image->getImgTop();
  //unsigned char segImg[25] =   {1,1,3,1,1,
			//	0,1,1,1,0,
			//	0,3,1,3,0,
			//	2,1,1,1,2,
			//	2,1,1,1,0};
//  int dy = 10;
  for (int y=0; y < iparams_.height; y+=dy) {
    //cout << y << endl;
    //cout << "SegIMG: " << y<< " " <<(int) segImg[y * iparams_.width] << endl;
    rows.push_back(findRunsInRow(segImg, y));
  }
  return rows;
}

vector<Node*> BlobDetector::findRunsInRow(unsigned char *segImg, int y) {
  vector<Node*> runs;
  int start = 0;
  //int dx = 10;
  char runColor = segImg[iparams_.width * y];
  char color;
  for (int x=0; x <= iparams_.width; x+=dx) { 
    if (x < iparams_.width) 
      color = segImg[iparams_.width * y + x];
    else
      color = c_UNDEFINED;
    if (runColor != color){// or x == width) {
      int end = x - 1;
//      if (x == width -1 && runColor == color)
  //      end = x;
    //  else if ()
      if (runColor != c_UNDEFINED){
        Run *run = new Run(start, end, runColor);
        run->yi = y;
        run->yf = y;
        run->xi = start;
        run->xf = end;
        run->color_ct += end - start;
        Node* node = new Node(run);
        runs.push_back(node);
      }
      start = x;
      runColor = color;
    }
  }
  return runs;
}

void BlobDetector::mergeRuns(vector<vector<Node*>> &rows) {
  for (int y=dy; y<iparams_.height;y+=dy) {
    //cout << "Y: " << y << endl;
    vector<Node*> firstRow = rows[y/dy - 1];
    vector<Node*> secondRow = rows[y/dy];
    int i=0;
    int j=0;
    while (i < firstRow.size() and j < secondRow.size()) {
      Node *a = firstRow[i];
      Node *b = secondRow[j];
      if (a->data->color == b->data->color) {
        //a->merge(b);
        if (b->parent != b) {// already has a parent
          a->parent = b->parent;
          if (a->data->end > b->parent->data->xf)
            b->parent->data->xf = a->data->end;
          b->parent->data->color_ct += a->data->color_ct;
        }
        else {// does not have a parent
          b->parent = a->parent;
          if (b->data->end > a->parent->data->xf)
            a->parent->data->xf = b->data->end;
          if (b->data->start < a->parent->data->xi)
            a->parent->data->xi = b->data->start;
          a->parent->data->yf = b->data->yi;
          a->parent->data->color_ct += b->data->color_ct;
        }
      }
      if (a->data->end < b->data->end) {
        i++;
      } else {
        j++;
      }
    } 
  } 
}
