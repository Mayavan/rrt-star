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

#include "RRTStarPlanner.hpp"
#include "MapManager.hpp"
#include "Node.hpp"

#define DEBUG false

RRTStarPlanner::RRTStarPlanner(std::string fileLocation, int step_size,
                               long minimumIteration) {
  MapManager manager(fileLocation);

  branchLength = step_size;
  regionRadius = step_size * 3 / 2;
  distanceToTarget = branchLength + 1;
  minIteration = minimumIteration;
  map = manager;
}

// Return a random point from the Cspace
std::pair<int, int> RRTStarPlanner::getRandomPoint() {
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_int_distribution<> x(0, 408);
  std::uniform_int_distribution<> y(0, 408);
  std::pair<int, int> randomPoint(x(gen), y(gen));

  return randomPoint;
}

bool RRTStarPlanner::hasObstacle(std::pair<int, int> Xnear,
                                 std::pair<int, int> Xnew) {
  bool result = false;

  float diff1 = (Xnew.first - Xnear.first);
  float diff2 = (Xnew.second - Xnear.second);

  int decimatedIndex;
  float diff;

  // take the greater difference
  if (fabs(diff1) > fabs(diff2)) {
    diff = diff1;
    decimatedIndex = 1;
  } else {
    diff = diff2;
    decimatedIndex = 0;
  }

  // Creates set of points between two points
  std::vector<std::vector<float> > pointsToCheck;
  std::vector<float> nearPoint;
  nearPoint.push_back(Xnear.first);
  nearPoint.push_back(Xnear.second);

  pointsToCheck.push_back(nearPoint);

  for (int ii = 1; ii <= fabs(diff); ii++) {
    std::vector<float> point;
    point.push_back(Xnear.first + ii * diff1 / fabs(diff));
    point.push_back(Xnear.second + ii * diff2 / fabs(diff));

    point[decimatedIndex] = floor(point[decimatedIndex]);
    pointsToCheck.push_back(point);

    if (floor(point[decimatedIndex]) != point[decimatedIndex]) {
      point[decimatedIndex]++;
      pointsToCheck.push_back(point);
    }
  }

  // returns true if one of the point in between is an obstacle
  for (auto const& value : pointsToCheck) {
    std::pair<int, int> floatVec(floor(value[0]), floor(value[1]));
    if (map.checkObstacle(floatVec)) {
      result = true;
    }
  }

  return result;
}

// returns the nearest node in the tree
std::pair<int, int> RRTStarPlanner::findNearest(std::pair<int, int>& Xrand) {
  std::pair<int, int> Xnear;
  long min_distance = 1000;
  long distance;

  for (auto const& value : nodes) {
    distance = calculateDistance(Xrand, value.nodePosition);
    if (distance < min_distance) {
      min_distance = distance;
      Xnear = value.nodePosition;
    }
  }
  return Xnear;
}

// returns the new node displaced a particular branch length in the random node's direction
std::pair<int, int> RRTStarPlanner::newNode(std::pair<int, int> Xnear,
                                            std::pair<int, int> Xrand) {
  std::pair<int, int> Xnew;
  float slope = (Xrand.second - Xnear.second) / (Xrand.first - Xnear.first);
  float adjuster = branchLength * sqrt(1 / (1 + pow(slope, 2)));

  std::pair<int, int> point1(Xnear.first + adjuster,
                             Xnear.second + slope * adjuster);
  std::pair<int, int> point2(Xnear.first - adjuster,
                             Xnear.second - slope * adjuster);

  float distance1 = calculateDistance(Xrand, point1);
  float distance2 = calculateDistance(Xrand, point2);
  if (distance1 < distance2)
    Xnew = point1;
  else
    Xnew = point2;

  return Xnew;
}

// returns the index of nodes in neighbourhood
std::vector<int> RRTStarPlanner::getNeighbourhood(std::pair<int, int> Xnew) {
  std::vector<int> neighbourhood;
  unsigned index = 0;
  for (auto const& value : nodes) {
    if (calculateDistance(value.nodePosition, Xnew) < regionRadius)
      neighbourhood.push_back(index);
    index++;
  }
  return neighbourhood;
}

// returns the parent with least cost to come
std::vector<int> RRTStarPlanner::getBestParent(std::vector<int> neighbourhood) {
  float min = nodes[neighbourhood.front()].costToCome;
  std::pair<int, int> Xnear = nodes[neighbourhood.front()].nodePosition;
  int position = neighbourhood[0];
  for (auto const& value : neighbourhood) {
    if (min > nodes[value].costToCome) {
      min = nodes[value].costToCome;
      Xnear = nodes[value].nodePosition;
      position = value;
    }
  }

  std::vector<int> output;
  output.emplace_back(Xnear.first);
  output.push_back(Xnear.second);
  output.push_back(position);
// The third index is the position in the tree
  return output;
}

// returns the position of the parent
long RRTStarPlanner::findParent(long position_of_child) {
  unsigned index = 0;
  for (auto const& i : nodes) {
    for (auto const& j : i.branches) {
      if (j == position_of_child)
        return index;
    }
    index++;
  }
  return 0;
}

// returns random number between min and max
int RRTStarPlanner::randNum(int min, int max) {
  return rand() % max + min;
}

// returns the euclidian distance between two points
float RRTStarPlanner::calculateDistance(std::pair<int, int> first_point,
                                        std::pair<int, int> second_point) {
  return (float) sqrt(
      (double) pow(first_point.first - second_point.first, 2)
          + (double) pow(first_point.second - second_point.second, 2));
}

std::vector<std::pair<int, int> > RRTStarPlanner::plan(
    std::pair<int, int> root, std::pair<int, int> target) {
  std::vector<std::pair<int, int> > plan;

  Node twig;
  twig.nodePosition = root;
  twig.costToCome = 0;
  nodes.push_back(twig);

  if (map.checkObstacle(target)) {
    std::cout << "Target in obstacle" << std::endl;
    return plan;
  }

  int count = 0;

  std::vector<int> neighbourhood, parent;
  std::pair<int, int> Xnew, Xnear, Xnearest, Xrand;
  int position;

  Xnew = root;
  std::cout << "------------- Searching ------------- " << std::endl;
  while ((distanceToTarget > branchLength)
      || count < minIteration) {
    count++;

    if (DEBUG)
      std::cout << "-- Current Count :" << count
                << "    -- distance to target :" << distanceToTarget
                << "    -- Has Obstacle :" << hasObstacle(target, Xnew)
                << std::endl;

    Xrand = getRandomPoint();

    Xnearest = findNearest(Xrand);
    if (Xnearest.first == Xrand.first || Xnearest.second == Xrand.second)
      continue;
    Xnew = newNode(Xnearest, Xrand);
    neighbourhood = getNeighbourhood(Xnew);
    parent = getBestParent(neighbourhood);
    Xnear.first = parent[0];
    Xnear.second = parent[1];
    position = parent[2];

    // Add node if obstacle not in between
    if (!hasObstacle(Xnear, Xnew)) {
      long current_no_of_nodes = nodes.size();

      // Add new node
      Node temp;

      temp.nodePosition = Xnew;
      temp.costToCome = nodes[position].costToCome
          + calculateDistance(Xnear, Xnew);

      nodes.push_back(temp);
      // Add child location in parent node
      nodes[position].branches.push_back(current_no_of_nodes);

      // check if close to target
      distanceToTarget = calculateDistance(Xnew, target);

      // Rewiring
      for (auto const& value : neighbourhood) {
        if ((nodes[current_no_of_nodes].costToCome
            + calculateDistance(nodes[value].nodePosition, Xnew))
            < nodes[value].costToCome) {
          // If cost from new node is cheaper switch parents
          long location = findParent(value);
          nodes[location].branches.erase(
              std::remove(nodes[location].branches.begin(),
                          nodes[location].branches.end(), value),
              nodes[location].branches.end());
          nodes[current_no_of_nodes].branches.push_back(value);
        }
      }
    }
  }  // end of search loop

  // Adding the target Node to the tree
  long current_no_of_nodes = nodes.size();
  // Add new node
  Node temp;

  temp.nodePosition = target;
  temp.costToCome = nodes[current_no_of_nodes - 1].costToCome
      + calculateDistance(target, Xnew);

  nodes.push_back(temp);

  std::cout << "Completed Search" << std::endl;

  // Add child location in parent node
  nodes[current_no_of_nodes - 1].branches.push_back(current_no_of_nodes);

  std::cout << "------------- Search Optimal Path ------------- " << std::endl;
  // Track the optimal path
  long node_number = current_no_of_nodes;
  std::pair<int, int> current_node = target;

  while (root != current_node) {

    plan.push_back(current_node);

    node_number = findParent(node_number);
    current_node = nodes[node_number].nodePosition;
  }
  plan.push_back(root);

  return plan;
}

void RRTStarPlanner::plotPlan(std::vector<std::pair<int, int> > plan) {
  std::cout << "Plotting MAP with Global Plan" << std::endl;
  map.plotImage(plan);
  map.showImage();
}
