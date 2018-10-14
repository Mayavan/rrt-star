/** @file test.cpp
 *
 * @brief This file tests the functions in the Planner classes.
 *
 * @author RajendraMayavan
 * @copyright 2018 , MIT License

*/

#include <gtest/gtest.h>
#include <vector>
#include "MapManager.hpp"
#include "RRTStarPlanner.hpp"

// Tests for class RRTStarPlanner
TEST(calculateDistance, should_pass) {
  RRTStarPlanner ob("/test.jpg", 5);
  std::pair<int, int> firstPoint(0, 0);
  std::pair<int, int> secondPoint(3, 4);
  EXPECT_EQ(5, ob.calculateDistance(firstPoint, secondPoint));
}

TEST(plan, should_pass) {
  RRTStarPlanner ob("/test.jpg", 5);
  std::pair<int, int> firstPoint(15, 15);
  std::pair<int, int> secondPoint(13, 14);
  std::vector<std::pair<int, int> > result = ob.plan(firstPoint, secondPoint);
  EXPECT_EQ(firstPoint, result.front());
  EXPECT_EQ(firstPoint, result.back());
}

// Tests for class MapManager
TEST(getState, should_pass) {
  MapManager ob("/test.jpg");
  EXPECT_LT(150, ob.getState(0, 0));
}

TEST(checkObstacle, TestPass)
{
  std::pair<int, int> firstPoint(2, 2);
  MapManager ob("/test.jpg");
  EXPECT_TRUE(!ob.checkObstacle(firstPoint));
}

