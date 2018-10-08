#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

class MapManager {
 public:
  MapManager();
  int getState(int x, int y);
  void showImage();
  bool checkObstacle(std::vector<int> grid);
  std::vector<float> computeGridCoordinate(std::vector<float> position);
  std::vector<double> computeDistanceCoordinate(std::vector<float> position);
  std::vector<std::vector<int> > getCfree();

 private:
  cv::Mat image;
  std::vector<std::vector<int> > Cfree;

};
