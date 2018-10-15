#include "RRTStarPlanner.hpp"

#define DEBUG false
#define STEP_SIZE 6

int main(void) {
  // Initialization
  RRTStarPlanner planner(
      "../DemoFiles/maze.png", STEP_SIZE,
      10000);

  // Initialize position
  std::pair<int, int> start_point, target_point;

  start_point.first = 205;
  start_point.second = 1;

  target_point.first = 125;
  target_point.second = 405;

  if (DEBUG) {
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

  if (plan.empty()) {
    std::cout << "Exiting program" << std::endl;
    return 1;
  }

  planner.plotPlan(plan);  // Plot the plan

  std::cout << "Completed execution" << std::endl;

  return 0;
}
