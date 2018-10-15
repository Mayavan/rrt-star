#include "MapManager.hpp"

MapManager::MapManager() {

}

MapManager::MapManager(std::string fileLocation) {
  image = cv::imread(fileLocation, cv::IMREAD_GRAYSCALE);

  if (!image.data) {
    std::cout << "No Data to read" << std::endl;
  } else {
    std::cout << "Image read in map Manager" << std::endl;
  }

  // create Cfree space to produce random nodes
  for (int i = 0; i < 408; i++)
    for (int j = 0; j < 408; j++)
      if (MapManager::getState(i, j) > 150) {
        std::pair<int, int> point(i, j);
        Cfree.push_back(point);
      }
}

// returns the color at the given coordinate
int MapManager::getState(int x, int y) {
  return (int) image.at<uchar>((408 - y), x);
}

// Check if the particular grid has obstacle
bool MapManager::checkObstacle(std::pair<int, int> grid) {
  if (getState(grid.first, grid.second) == 0)
    return true;
  else
    return false;
}

// Plot on image
void MapManager::plotImage(std::vector<std::pair<int, int> > plan) {
  auto it = plan.begin();
  while (it != plan.end()) {
    cv::Scalar blue = cv::Scalar(255, 0, 0);
    cv::Point start = cv::Point(it->first, 408 - it->second);
    if ((++it) == plan.end())
      break;
    cv::line(image, start, cv::Point(it->first, 408 - it->second),
             CV_RGB(128, 128, 128),
             2, 0);
  }
}

// show the read image in a window
void MapManager::showImage() {
  namedWindow("Display window", cv::WINDOW_AUTOSIZE);  // Create a window for display.
  imshow("Display window", image);
  cv::waitKey(60000);
}
