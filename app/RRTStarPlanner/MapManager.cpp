#include "MapManager.hpp"

using namespace cv;

MapManager::MapManager() {

}

MapManager::MapManager(std::string fileLocation)
{
  image = imread(fileLocation, cv::IMREAD_GRAYSCALE);

    if (!image.data)
    {
        std::cout << "No Data " << std::endl;
    }

    // create Cfree space to produce random nodes
  for (int i = 0; i < 408; i++)
    for (int j = 0; j < 408; j++)
      if (MapManager::getState(i, j) > 150)
            {
        std::pair<int, int> point(i, j);
                Cfree.push_back(point);
            }
}

// returns the color at the given coordinate
int MapManager::getState(int x, int y)
{
  return (int) image.at<uchar>((408 - y), x);
}

// Check if the particular grid has obstacle
bool MapManager::checkObstacle(std::pair<int, int> grid)
{
  if (getState(grid.first, grid.second) == 0)
        return true;
    else
        return false;
}

// show the read image in a window
void MapManager::showImage()
{
    namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
    imshow("Display window", image);
    waitKey(60000);
}
