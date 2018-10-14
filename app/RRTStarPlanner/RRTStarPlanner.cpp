#include "RRTStarPlanner.hpp"
#include "MapManager.hpp"
#include "Node.hpp"

#define DEBUG false

RRTStarPlanner::RRTStarPlanner(std::string fileLocation, int step_size)
{
  MapManager manager(fileLocation);

  branchLength = step_size;
  regionRadius = step_size * 3 / 2;
  distanceToTarget = branchLength + 1;

  map = manager;
}

// Return a random point from the Cspace
std::pair<int, int> RRTStarPlanner::getRandomPoint()
{
    std::random_device rd;
    std::mt19937 gen(rd());

  std::uniform_int_distribution<> x(0, 408);
  std::uniform_int_distribution<> y(0, 408);
  std::pair<int, int> randomPoint(x(gen), y(gen));

  return randomPoint;
}

bool RRTStarPlanner::hasObstacle(std::pair<int, int> Xnear,
                                 std::pair<int, int> Xnew)
{
    bool result = false;

  float diff1 = (Xnew.first - Xnear.first);
  float diff2 = (Xnew.second - Xnear.second);

  int decimatedIndex;
    float diff;

    // take the greater difference
    if (fabs(diff1) > fabs(diff2))
    {
        diff = diff1;
    decimatedIndex = 1;
    }
    else
    {
        diff = diff2;
    decimatedIndex = 0;
    }

    // Creates set of points between two points
  std::vector<std::vector<float> > pointsToCheck;
  std::vector<float> nearPoint;
  nearPoint.push_back(Xnear.first);
  nearPoint.push_back(Xnear.second);

  pointsToCheck.push_back(nearPoint);

    for (int ii = 1; ii <= fabs(diff); ii++)
    {
        std::vector<float> point;
    point.push_back(Xnear.first + ii * diff1 / fabs(diff));
    point.push_back(Xnear.second + ii * diff2 / fabs(diff));

    point[decimatedIndex] = floor(point[decimatedIndex]);
    pointsToCheck.push_back(point);

    if (floor(point[decimatedIndex]) != point[decimatedIndex])
        {
      point[decimatedIndex]++;
      pointsToCheck.push_back(point);
        }
    }

    // returns true if one of the point in between is an obstacle
  for (int jj = 0; jj < pointsToCheck.size(); jj++)
    {
    std::pair<int, int> floatVec(floor(pointsToCheck[jj][0]),
                                 floor(pointsToCheck[jj][1]));
        if (map.checkObstacle(floatVec))
        {
            result = true;
        }
    }

    return result;
}

// returns the nearest node in the tree
std::pair<int, int> RRTStarPlanner::findNearest(std::pair<int, int> Xrand)
{
  std::pair<int, int> Xnear;
    long min_distance = 1000;
    long distance;

  for (int ii = 0; ii < nodes.size(); ii++)
    {
    distance = calculateDistance(Xrand, nodes[ii].nodePosition);
        if (distance < min_distance)
        {
            min_distance = distance;
      Xnear = nodes[ii].nodePosition;
        }
    }
    return Xnear;
}

// returns the new node displaced a particular branch length in the random node's direction
std::pair<int, int> RRTStarPlanner::newNode(std::pair<int, int> Xnear,
                                            std::pair<int, int> Xrand)
{
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
std::vector<int> RRTStarPlanner::getNeighbourhood(std::pair<int, int> Xnew)
{
    std::vector<int> neighbourhood;
  for (int i = 0; i < nodes.size(); i++)
    {
    if (calculateDistance(nodes[i].nodePosition, Xnew) < regionRadius)
            neighbourhood.push_back(i);
    }
    return neighbourhood;
}

// returns the parent with least cost to come
std::vector<int> RRTStarPlanner::getBestParent(
    std::vector<int> neighbourhood) {
  float min = nodes[neighbourhood[0]].costToCome;
  std::pair<int, int> Xnear = nodes[neighbourhood[0]].nodePosition;
  int position = neighbourhood[0];
  for (int i = 1; i < neighbourhood.size(); i++) {
    if (min > nodes[neighbourhood[i]].costToCome)
{
      min = nodes[neighbourhood[i]].costToCome;
      Xnear = nodes[neighbourhood[i]].nodePosition;
      position = neighbourhood[i];
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
long RRTStarPlanner::findParent(long position_of_child)
{
  for (long i = 0; i < nodes.size(); i++) {
    for (int j = 0; j < nodes[i].branches.size(); j++) {
      if (nodes[i].branches[j] == position_of_child)
                return i;
        }
    }
  return 0;
}

// returns random number between min and max
int RRTStarPlanner::randNum(int min, int max)
{
    return rand() % max + min;
}

// returns the euclidian distance between two points
float RRTStarPlanner::calculateDistance(std::pair<int, int> first_point,
                                        std::pair<int, int> second_point)
{
  return (float) sqrt(
      (double) pow(first_point.first - second_point.first, 2)
          + (double) pow(first_point.second - second_point.second, 2));
}

std::vector<std::pair<int, int> > RRTStarPlanner::plan(
    std::pair<int, int> root, std::pair<int, int> target)
{
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
    std::cout << "------------- Starting Search ------------- " << std::endl;
  while ((distanceToTarget > branchLength) || hasObstacle(target, Xnew)
      || count < 10000)
    {
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
    if (!hasObstacle(Xnear, Xnew))
        {
      long current_no_of_nodes = nodes.size();

            if (DEBUG)
            {
                std::cout << "Printing nodes...\n";
        for (int cc = 0; cc < nodes.size(); cc++)
                {
          std::cout << "[" << nodes[cc].nodePosition.first << ", "
                    << nodes[cc].nodePosition.second << "] " << std::endl;
                }
                std::cout << "Press any key to continue...\n";
                getchar();
            }

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
            for (int cc = 0; cc < neighbourhood.size(); cc++)
            {
        if ((nodes[current_no_of_nodes].costToCome
            + calculateDistance(nodes[neighbourhood[cc]].nodePosition, Xnew))
            < nodes[neighbourhood[cc]].costToCome)
                {
                    // If cost from new node is cheaper switch parents
                    long location = findParent(neighbourhood[cc]);
          nodes[location].branches.erase(
              std::remove(nodes[location].branches.begin(),
                          nodes[location].branches.end(), neighbourhood[cc]),
              nodes[location].branches.end());
          nodes[current_no_of_nodes].branches.push_back(neighbourhood[cc]);
                }
            }
        }
    } // end of search loop

    // Adding the target Node to the tree
  long current_no_of_nodes = nodes.size();
    // Add new node
  Node temp;

  temp.nodePosition = target;
  temp.costToCome = nodes[current_no_of_nodes - 1].costToCome
      + calculateDistance(target, Xnew);

  nodes.push_back(temp);

    // Add child location in parent node
  nodes[current_no_of_nodes - 1].branches.push_back(current_no_of_nodes);

    std::cout << "------------- Search Optimal Path ------------- " << std::endl;
    // Track the optimal path
    long node_number = current_no_of_nodes;
  std::pair<int, int> current_node = target;

  while (root.first != current_node.first || root.second != current_node.second)
    {
    std::pair<int, int> pos;
    pos.first = ((current_node.first * 0.05) - 10);
    pos.second = ((current_node.second * 0.05) - 10);

        plan.push_back(pos);

        node_number = findParent(node_number);
    current_node = nodes[node_number].nodePosition;
    }

    return plan;
}
