#include <VisionWindow.h>

#define MIN_PEN_WIDTH 3
#define IS_RUNNING_CORE (core_ && core_->vision_ && ((UTMainWnd*)parent_)->runCoreRadio->isChecked())

void VisionWindow::redrawImages() {
  if(DEBUG_WINDOW) std::cout << "redrawImages\n";
  if(!enableDraw_) return;

  if (((UTMainWnd*)parent_)->streamRadio->isChecked()) {
    int ms = timer_.elapsed();
    if(ms < MS_BETWEEN_FRAMES)
      return;
    timer_.start();
  }
  setImageSizes();

  redrawImages(rawImageTop,    segImageTop,    objImageTop,    horizontalBlobImageTop,    verticalBlobImageTop,    transformedImageTop);
  redrawImages(rawImageBottom, segImageBottom, objImageBottom, horizontalBlobImageBottom, verticalBlobImageBottom, transformedImageBottom);

  updateBigImage();
}

void VisionWindow::updateBigImage(ImageWidget* source) {
  ImageProcessor* processor = getImageProcessor(source);
  int width = processor->getImageWidth(), height = processor->getImageHeight();
  bigImage->setImageSource(source->getImage(), width, height);
}

void VisionWindow::updateBigImage() {
  switch(currentBigImageType_) {
    case RAW_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(rawImageTop);
      else
        updateBigImage(rawImageBottom);
        break;
    case SEG_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(segImageTop);
      else
        updateBigImage(segImageBottom);
        break;
    case OBJ_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(objImageTop);
      else
        updateBigImage(objImageBottom);
        break;
    case HORIZONTAL_BLOB_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(horizontalBlobImageTop);
      else
        updateBigImage(horizontalBlobImageBottom);
        break;
    case VERTICAL_BLOB_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(verticalBlobImageTop);
      else
        updateBigImage(verticalBlobImageBottom);
        break;
    case TRANSFORMED_IMAGE:
      if (currentBigImageCam_ == IMAGE_TOP)
        updateBigImage(transformedImageTop);
      else
        updateBigImage(transformedImageBottom);
        break;
  }

  // draw all pixels of seg image in big window
  if (currentBigImageType_ == SEG_IMAGE){
    drawSegmentedImage(bigImage);
    if (overlayCheck->isChecked()) {
      drawBall(bigImage);
      drawGoal(bigImage);
      drawBallCands(bigImage);
      drawBeacons(bigImage);
    }
  }

  bigImage->update();

}

void VisionWindow::redrawImages(ImageWidget* rawImage, ImageWidget* segImage, ImageWidget* objImage, ImageWidget* horizontalBlobImage, ImageWidget* verticalBlobImage, ImageWidget* transformedImage) {
  drawRawImage(rawImage);
  drawSmallSegmentedImage(segImage);

  objImage->fill(0);
  drawBall(objImage);

  if(horizonCheck->isChecked()) {
    drawHorizonLine(rawImage);
    drawHorizonLine(segImage);
    drawHorizonLine(horizontalBlobImage);
    drawHorizonLine(verticalBlobImage);
  }

  // if overlay is on, then draw objects on the raw and seg image as well
  if (overlayCheck->isChecked()) {
    drawBall(rawImage);
    drawGoal(rawImage);
    drawBallCands(rawImage);
    drawBeacons(rawImage);

    drawBall(segImage);
    drawGoal(segImage);
    drawBallCands(segImage);
    drawBeacons(segImage);
  }

  drawBall(verticalBlobImage);
  drawBallCands(verticalBlobImage);

  transformedImage->fill(0);

  rawImage->update();
  segImage->update();
  objImage->update();
  horizontalBlobImage->update();
  verticalBlobImage->update();
  transformedImage->update();
}

void VisionWindow::drawRawImage(ImageWidget* widget) {
  ImageProcessor* processor = getImageProcessor(widget);
  unsigned char* image = processor->getImg();
  const ImageParams& iparams = processor->getImageParams();
  const CameraMatrix& cmatrix = processor->getCameraMatrix();
  if (!processor->isImageLoaded()) {
    widget->fill(0);
    return;
  }
  for (int y = 0; y < iparams.height; y++) {
    for (int x = 0; x < iparams.width; x+=2) {

      color::Yuv422 yuyv;
      yuyv.y0 = (int) (*(image++));
      yuyv.u = (int) (*(image++));
      yuyv.y1 = (int) (*(image++));
      yuyv.v = (int) (*(image++));


      color::Rgb rgb1, rgb2;
      color::yuv422ToRgb(yuyv, rgb1, rgb2);

      // First pixel
      QRgb value1 = qRgb(rgb1.r, rgb1.g, rgb1.b);
      widget->setPixel(x, y, value1);

      // Second Pixel
      QRgb value2 = qRgb(rgb2.r, rgb2.g, rgb2.b);
      Coordinates u = cmatrix.undistort(x, y);
      widget->setPixel(u.x + 1, u.y, value2);

    }
  }

}

void VisionWindow::drawSmallSegmentedImage(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();
  unsigned char* segImg = processor->getSegImg();
  int hstep = 4, vstep = 2;
  if (robot_vision_block_ == NULL || segImg == NULL) {
    image->fill(0);
    return;
  }

  // This will be changed on the basis of the scan line policy
  for (int y = 0; y < iparams.height; y+=vstep) {
    for (int x = 0; x < iparams.width; x+=hstep) {
      int c = segImg[iparams.width * y + x];
      for (int smallY = 0; smallY < vstep; smallY++) {
        for (int smallX = 0; smallX < hstep; smallX++) {
          image->setPixel(x + smallX, y + smallY, segRGB[c]);
        }
      }
    }
  }
}

void VisionWindow::drawSegmentedImage(ImageWidget *image) {
  ImageProcessor* processor = getImageProcessor(image);
  const ImageParams& iparams = processor->getImageParams();
  if (doingClassification_) {
    if (image_block_ == NULL) {
      image->fill(0);
      return;
    }

    // Classify the entire image from the raw image
    unsigned char *rawImg = processor->getImg();
    unsigned char* colorTable = processor->getColorTable();
    const ImageParams& iparams = processor->getImageParams();

    for (uint16_t y = 0; y < iparams.height; y++) {
      for (uint16_t x = 0; x < iparams.width; x++) {
        Color c = ColorTableMethods::xy2color(rawImg, colorTable, x, y, iparams.width);
        image->setPixel(x, y, segRGB[c]);
      }
    }
  }
  else {
    unsigned char* segImg = processor->getSegImg();
    if (robot_vision_block_ == NULL || segImg == NULL) {
      image->fill(0);
      return;
    }

    // Seg image from memory
    for (int y = 0; y < iparams.height; y++) {
      for (int x = 0; x < iparams.width; x++) {
        int c = segImg[iparams.width * y + x];
        image->setPixel(x, y, segRGB[c]);
      }
    }
  }
  if(horizonCheck->isChecked())
    drawHorizonLine(image);
}

void VisionWindow::drawBall(ImageWidget* image) {

  QPainter painter(image->getImage());
  painter.setPen(QPen(QColor(0, 255, 127), 3));
  if(IS_RUNNING_CORE) { // Added ! to force draw ball
    ImageProcessor* processor = getImageProcessor(image);
//    std::cout << "Running core" << std::endl;
//    BallCandidate* best = processor->getBestBallCandidate();
    WorldObject* ball = &world_object_block_->objects_[WO_BALL];
    if(!ball->seen) return;
    if( (ball->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!ball->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    //std::cout << "DrawBall: " << ball->imageCenterX << " " << ball->imageCenterY << " " << ball->radius << std::endl;
    int radius = ball->radius;
    painter.drawEllipse(ball->imageCenterX - radius, ball->imageCenterY - radius, radius * 2, radius * 2);
//    int r = best->radius;
//    painter.drawEllipse(
//      (int)best->centerX - r - 1,
//      (int)best->centerY - r - 1, 2 * r + 2, 2 * r + 2);
  }
  else if (world_object_block_ != NULL) {
    WorldObject* ball = &world_object_block_->objects_[WO_BALL];
    if(!ball->seen) return;
    if( (ball->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!ball->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    //std::cout << "DrawBall: " << ball->imageCenterX << " " << ball->imageCenterY << " " << ball->radius << std::endl;
    int radius = ball->radius;
    painter.drawEllipse(ball->imageCenterX - radius, ball->imageCenterY - radius, radius * 2, radius * 2);
  }
}

void VisionWindow::drawGoal(ImageWidget* image) {
  QPainter painter(image->getImage());
  painter.setPen(QPen(QColor(0, 255, 127), 3));
  auto processor = getImageProcessor(image);
  const auto& cmatrix = processor->getCameraMatrix();
  if(IS_RUNNING_CORE) { 
    ImageProcessor* processor = getImageProcessor(image);
//    std::cout << "Running core" << std::endl;
    WorldObject* goal = &world_object_block_->objects_[WO_UNKNOWN_GOAL];
    if(!goal) return;
    if( (goal->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!goal->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    if (!goal->seen) return;
    //int r = best->radius;
    int w = cmatrix.getCameraWidthByDistance(goal->visionDistance, 110);
    int h = cmatrix.getCameraHeightByDistance(goal->visionDistance, 100);

    painter.drawRect(
      (int)goal->imageCenterX - w - 1,
      (int)goal->imageCenterY - h - 1, 2 * w + 2, 2 * h + 2);
  }
  else if (world_object_block_ != NULL) {
    WorldObject* goal = &world_object_block_->objects_[WO_UNKNOWN_GOAL];
    if(!goal->seen) return;
    if( (goal->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!goal->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    //std::cout << "DrawBall: " << ball->imageCenterX << " " << ball->imageCenterY << " " << ball->radius << std::endl;
    int w = cmatrix.getCameraWidthByDistance(goal->visionDistance, 110);
    int h = cmatrix.getCameraHeightByDistance(goal->visionDistance, 100);

    painter.drawRect(goal->imageCenterX - w, goal->imageCenterY - h, w * 2, h * 2);
  }
}

void VisionWindow::drawBallCands(ImageWidget* image) {
}

void VisionWindow::drawHorizonLine(ImageWidget *image) {
  if (robot_vision_block_ && _widgetAssignments[image] == IMAGE_TOP) {
    HorizonLine horizon = robot_vision_block_->horizon;
    if (horizon.exists) {
      QPainter painter(image->getImage());
      QPen wpen = QPen(segCol[c_BLUE], MIN_PEN_WIDTH);
      painter.setPen(wpen);

      ImageProcessor* processor = getImageProcessor(image);
      const ImageParams& iparams = processor->getImageParams();

      int x1 = 0;
      int x2 = iparams.width - 1;
      int y1 = horizon.gradient * x1 + horizon.offset;
      int y2 = horizon.gradient * x2 + horizon.offset;
      painter.drawLine(x1, y1, x2, y2);
    }
  }
}

void VisionWindow::drawWorldObject(ImageWidget* image, QColor color, int worldObjectID) {
  if (world_object_block_ != NULL) {
    QPainter painter(image->getImage());
    QPen wpen = QPen(color, 5);   // 2
    painter.setPen(wpen);
    WorldObject* object = &world_object_block_->objects_[worldObjectID];
    if(!object->seen) return;
    if( (object->fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) ||
        (!object->fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) ) return;
    int offset = 10;      // 5
    int x1, y1, x2, y2;

    x1 = object->imageCenterX - offset,
    y1 = object->imageCenterY - offset,
    x2 = object->imageCenterX + offset,
    y2 = object->imageCenterY + offset;

    painter.drawLine(x1, y1, x2, y2);

    x1 = object->imageCenterX - offset,
    y1 = object->imageCenterY + offset,
    x2 = object->imageCenterX + offset,
    y2 = object->imageCenterY - offset;

    painter.drawLine(x1, y1, x2, y2);
  }
}

void VisionWindow::drawBeacons(ImageWidget* image) {
  if(world_object_block_ == NULL) return;
  map<WorldObjectType,vector<QColor>> beacons = {
    { WO_BEACON_BLUE_YELLOW, { segCol[c_BLUE], segCol[c_YELLOW] } },
    { WO_BEACON_YELLOW_BLUE, { segCol[c_YELLOW], segCol[c_BLUE] } },
    { WO_BEACON_BLUE_PINK, { segCol[c_BLUE], segCol[c_PINK] } },
    { WO_BEACON_PINK_BLUE, { segCol[c_PINK], segCol[c_BLUE] } },
    { WO_BEACON_PINK_YELLOW, { segCol[c_PINK], segCol[c_YELLOW] } },
    { WO_BEACON_YELLOW_PINK, { segCol[c_YELLOW], segCol[c_PINK] } }
  };
  auto processor = getImageProcessor(image);
  const auto& cmatrix = processor->getCameraMatrix();
  QPainter painter(image->getImage());
  painter.setRenderHint(QPainter::Antialiasing);
  for(auto beacon : beacons) {
    auto& object = world_object_block_->objects_[beacon.first];
    if(!object.seen) continue;
    if(object.fromTopCamera && _widgetAssignments[image] == IMAGE_BOTTOM) continue;
    if(!object.fromTopCamera && _widgetAssignments[image] == IMAGE_TOP) continue;
    QPen tpen(beacon.second[0]), bpen(beacon.second[1]);

    int width = cmatrix.getCameraWidthByDistance(object.visionDistance, 110);
    int height = cmatrix.getCameraHeightByDistance(object.visionDistance, 100);
    int x1 = object.imageCenterX - width / 2;
    
    // Draw top
    int ty1 = object.imageCenterY - height;
    QPainterPath tpath;
    tpath.addRoundedRect(QRect(x1, ty1, width, height), 5, 5);
    painter.setPen(tpen);
    painter.fillPath(tpath, QBrush(beacon.second[0]));

    // Draw bottom
    int by1 = object.imageCenterY, by2 = object.imageCenterY + height;
    QPainterPath bpath;
    bpath.addRoundedRect(QRect(x1, by1, width, height), 5, 5);
    painter.setPen(bpen);
    painter.fillPath(bpath, QBrush(beacon.second[1]));
  }
}
