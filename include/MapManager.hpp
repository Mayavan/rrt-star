#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <string>

class MapManager {
 public:
  MapManager();
  MapManager(std::string fileLocation);
  int getState(int x, int y);
  void showImage();
  bool checkObstacle(std::pair<int, int> grid);
  std::vector<std::vector<int> > getCfree();

 private:
  cv::Mat image;
  std::vector<std::pair<int, int> > Cfree;
};
