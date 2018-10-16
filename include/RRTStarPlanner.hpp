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
#ifndef INCLUDE_RRTSTARPLANNER_HPP_
#define INCLUDE_RRTSTARPLANNER_HPP_

#include <math.h>
#include <vector>
#include <string>
#include <utility>
#include <algorithm>

#include "MapManager.hpp"
#include "Node.hpp"

class RRTStarPlanner {
 public:
  RRTStarPlanner(const std::string& fileLocation, const int& stepSize,
                 const int64& minimumIteration);
  std::vector<std::pair<int, int> > plan(const std::pair<int, int>& root,
                                         const std::pair<int, int>& target);
  float calculateDistance(const std::pair<int, int>& firstPoint,
                          const std::pair<int, int>& secondPoint);
  void plotPlan(const std::vector<std::pair<int, int> >& plan);

 private:
  float regionRadius;
  int branchLength;
  float distanceToTarget;
  int64 minIteration;
  std::vector<Node> nodes;
  MapManager map;

  std::pair<int, int> getRandomPoint();
  std::pair<int, int> findNearest(const std::pair<int, int>& Xrand);
  bool hasObstacle(const std::pair<int, int>& Xnear,
                   const std::pair<int, int>& Xnew);
  std::pair<int, int> newNode(const std::pair<int, int>& Xnear,
                              const std::pair<int, int>& Xrand);
  std::vector<int> getNeighbourhood(const std::pair<int, int>& Xnew);
  std::vector<int> getBestParent(const std::vector<int>& neighbourhood);
  int64 findParent(const int64& positionOfChild);
};

#endif  // INCLUDE_RRTSTARPLANNER_HPP_
