#include "MapManager.hpp"
#include "RRTStarPlanner.hpp"
#include "matplotlibcpp.h"
#include <iostream>
#include <fstream>
#include <algorithm>

#include <cmath>

#define DEBUG false
#define STEP_SIZE 6

namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
    // Initialization
  RRTStarPlanner planner("maze.PNG", STEP_SIZE);
  MapManager manager("maze.PNG");

  // Initialize position
  std::pair<int, int> start_point, target_point;

  start_point.first = 123;
  start_point.second = 7;
  ;

  target_point.first = 197;
  target_point.second = 404;


    if (DEBUG)
    {
    std::cout << "Start Point: " << start_point.first << ", "
              << start_point.second;
    std::cout << std::endl;
    std::cout << "End Point: " << target_point.first << ", "
              << target_point.second;
    std::cout << std::endl;
    }

    // Make the plan with RRT
  std::vector<std::pair<int, int>> plan;
  plan = planner.plan(start_point, target_point);

    int counter = 0;

  std::cout << "Plotting MAP with Global Plan";
  std::cout << std::endl;

    // Plot the map
  for (int i = 0; i < 408; i++)
    for (int j = 0; j < 408; j++)
      if (manager.getState(i, j) > 150)
            {
        std::vector<double> x, y;
                x.push_back((i * 0.05) - 10);
                x.push_back((i * 0.05) - 10 + 0.01);
                y.push_back((j * 0.05) - 10);
                y.push_back((j * 0.05) - 10 + 0.01);
                plt::plot(x, y);
                if (DEBUG)
          std::cout << "Printing" << counter++;
        std::cout << std::endl;
            }

    // Plot the plan
  std::vector<double> x, y;
    for (int i = plan.size() - 1; i >= 0; i--)
    {
    x.push_back(plan[i].first);
    y.push_back(plan[i].second);
    }
    plt::plot(x, y);
    plt::xlim(-10, 10);
    plt::ylim(-10, 10);
    plt::show();

    return 0;
}
