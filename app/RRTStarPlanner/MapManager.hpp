#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <string>

class MapManager {
 public:
  MapManager(std::string fileLocation);
  int getState(int x, int y);
  void showImage();
  bool checkObstacle(std::vector<int> grid);
  std::vector<std::vector<int> > getCfree();

 private:
  cv::Mat image;
  std::vector<std::vector<int> > Cfree;
};
