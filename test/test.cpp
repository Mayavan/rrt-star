/**
 * @file test.cpp
 * @author RajendraMayavan
 * @copyright MIT License
 *
 * @brief RRT Planner - testing
 *
 */

#include <gtest/gtest.h>
#include <vector>

#include "MapManager.hpp"
#include "RRTStarPlanner.hpp"

std::string fileLocation =
    "/home/mayavan/workspace/Midterm/rrt-star/app/maze.png";

// Tests for class RRTStarPlanner
TEST(calculateDistance, should_pass) {
  RRTStarPlanner ob(fileLocation, 5, 10000);
  std::pair<int, int> firstPoint(0, 0);
  std::pair<int, int> secondPoint(3, 4);
  EXPECT_EQ(5, ob.calculateDistance(firstPoint, secondPoint));
}

// Tests for class MapManager
TEST(getState, should_pass) {
  MapManager ob(fileLocation);
  EXPECT_LT(150, ob.getState(25, 25));
}

TEST(checkObstacle, should_pass)
{
  std::pair<int, int> firstPoint(205, 1);
  MapManager ob(fileLocation);
  EXPECT_TRUE(!ob.checkObstacle(firstPoint));
}

TEST(plan, should_pass) {
  RRTStarPlanner ob(fileLocation, 5, 10000);
  std::pair<int, int> firstPoint(25, 25);
  std::pair<int, int> secondPoint(25, 375);
  std::vector<std::pair<int, int> > result = ob.plan(firstPoint, secondPoint);
  std::cout << result.front().first << ", " << result.back().second;
  EXPECT_EQ(secondPoint, result.front());
  EXPECT_EQ(firstPoint, result.back());
}
