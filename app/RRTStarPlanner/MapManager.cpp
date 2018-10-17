/**
 * @file test.cpp
 * @brief RRT Planner - Class to manage map image information and plot the planned path
 * @author RajendraMayavan
 * @copyright MIT License

 * Copyright (c) 2018 Mayavan

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "MapManager.hpp"

/**
 * @brief Default constructor
 */
MapManager::MapManager() {
}

/**
 * @brief initializes the image and Cfree vector
 * @param fileLocation a string indicating the location of the map
 */
MapManager::MapManager(const std::string& fileLocation) {
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

/**
 * @brief Function to get each pixel value
 * @param x the x coordinate of the pixel
 * @param y the y coordinate of the pixel
 * @return the intensity at the given coordinate
 */
int MapManager::getState(const int& x, const int& y) {
  return static_cast<int>(image.at<uchar>((408 - y), x));
}

/**
 * @brief Check if the particular grid has obstacle
 * @param grid the coordinate of the pixel
 * @return true if it has obstacle else false
 */
bool MapManager::checkObstacle(const std::pair<int, int>& grid) {
  if (getState(grid.first, grid.second) == 0)
    return true;
  else
    return false;
}

/**
 * @brief Plot the path on the image
 * @param plan vector of points indicating the planned path
 * @return none
 */
void MapManager::plotImage(const std::vector<std::pair<int, int> >& plan) {
  auto it = plan.begin();
  while (it != plan.end()) {
    cv::Point start = cv::Point(it->first, 408 - it->second);
    if ((++it) == plan.end())
      break;

    // Draw lines based on the path
    cv::line(image, start, cv::Point(it->first, 408 - it->second),
             CV_RGB(128, 128, 128), 2, 0);
  }
  showImage();
}

/**
 * @brief Displays the image file
 */
void MapManager::showImage() {
  // Create a window for display.
  namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  imshow("Display window", image);
  cv::waitKey(5000);
}
