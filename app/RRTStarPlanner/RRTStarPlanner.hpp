#include <math.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>

#include "RRTStarPlanner/MapManager.hpp"
#include "RRTStarPlanner/Node.hpp"

class RRTStarPlanner
{
public:
  RRTStarPlanner(const MapManager& manager, int step_size);
  std::vector<geometry_msgs::PoseStamped> makePlan(std::vector<float> root, std::vector<float> target);
  float calculateDistance(std::vector<float> first_point, std::vector<float> second_point);

private:  
  std::vector<std::vector<int> > Cfree;
  float regionRadius;
  int branchLength;
  float distanceToTarget;
  std::vector<Node> nodes;
  MapManager map;

  std::vector<float> getRandomPoint();
  std::vector<float> findNearest(std::vector<float> Xrand);
  bool hasObstacle(std::vector<float> Xnear, std::vector<float> Xnew);
  std::vector<float> newNode(std::vector<float> Xnear, std::vector<float> Xrand);
  std::vector<int> getNeighbourhood(std::vector<float> Xnew);
  std::vector<float> getBestParent(std::vector<int> neighbourhood);
  long findParent(long position_of_child);

  int randNum(int min, int max);
};
