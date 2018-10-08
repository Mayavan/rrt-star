#include <math.h>
#include <vector>
#include <algorithm>

#include "RRTStarPlanner/MapManager.hpp"
#include "RRTStarPlanner/Node.hpp"

class RRTStarPlanner
{
 public:
  RRTStarPlanner(const MapManager& manager, int step_size);
  std::vector<std::vector<float> > plan(std::vector<float> root, std::vector<float> target);
  float calculateDistance(std::vector<float> firstPoint, std::vector<float> secondPoint);

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
  int64 findParent(int64 positionOfChild);
  int randNum(int min, int max);
};
