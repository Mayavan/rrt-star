/**
 * @file test.cpp
 * @brief RRT Planner - Class to plan the path from one point to another using RRT* algorithm
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
 *
 */

#include <math.h>
#include <vector>
#include <algorithm>

#include "MapManager.hpp"
#include "Node.hpp"

class RRTStarPlanner
{
 public:
  RRTStarPlanner(std::string fileLocation, int stepSize,
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
};
