#include <math.h>
#include <vector>
#include <algorithm>

#include "MapManager.hpp"
#include "Node.hpp"

class RRTStarPlanner
{
 public:
  RRTStarPlanner(std::string fileLocation, int step_size,
                 long minimumIteration);
  std::vector<std::pair<int, int> > plan(std::pair<int, int> root,
                                         std::pair<int, int> target);
  float calculateDistance(std::pair<int, int> firstPoint,
                          std::pair<int, int> secondPoint);
  void plotPlan(std::vector<std::pair<int, int> > plan);

 private:
  float regionRadius;
  int branchLength;
  float distanceToTarget;
  long minIteration;
  std::vector<Node> nodes;
  MapManager map;

  std::pair<int, int> getRandomPoint();
  std::pair<int, int> findNearest(std::pair<int, int>& Xrand);
  bool hasObstacle(std::pair<int, int> Xnear, std::pair<int, int> Xnew);
  std::pair<int, int> newNode(std::pair<int, int> Xnear,
                             std::pair<int, int> Xrand);
  std::vector<int> getNeighbourhood(std::pair<int, int> Xnew);
  std::vector<int> getBestParent(std::vector<int> neighbourhood);
  int64 findParent(int64 positionOfChild);
  int randNum(int min, int max);
};
