/**
 * @file main.cpp
 * @brief RRT Planner - Main function to test RRT* algorithm with the demo map
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

int main(void) {
  int stepSize = 5;
  int64 minimumIter = 10000;
  std::string fileLocation = "../DemoFiles/maze.png";

  // Initialization
  RRTStarPlanner planner(fileLocation, stepSize, minimumIter);

  // Initialize position
  std::pair<int, int> start_point, target_point;

  start_point.first = 205;
  start_point.second = 1;

  target_point.first = 125;
  target_point.second = 405;

  // Make the plan with RRTStarPlanner
  std::vector<std::pair<int, int> > plan;
  plan = planner.plan(start_point, target_point);

  if (plan.empty()) {
    std::cout << "Exiting program" << std::endl;
    return 1;
  }

  // Plot the plan
  planner.plotPlan(plan);

  std::cout << "Completed execution" << std::endl;

  return 0;
}
