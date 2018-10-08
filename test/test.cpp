/** @file test.cpp
 *
 * @brief This file tests the functions in the Planner classes.
 *
 * @author RajendraMayavan
 * @copyright 2018 , MIT License

*/

#include <gtest/gtest.h>
#include <vector>
#include "../app/RRTStarPlanner/MapManager.hpp"
#include "../app/RRTStarPlanner/RRTStarPlanner.hpp"

// Tests for class RRTStarPlanner
TEST(calculateDistance, should_pass) {
  RRTStarPlanner ob;
  std::vector<float> firstPoint;
  std::vector<float> secondPoint;
  firstPoint.emplace_back(0);
  firstPoint.push_back(0);
  secondPoint.emplace_back(3);
  secondPoint.push_back(4);
  EXPECT_EQ(5, ob.calculateDistance(firstPoint, secondPoint));
}

TEST(plan, should_pass) {
  RRTStarPlanner ob;
  std::vector<float> firstPoint;
  std::vector<float> secondPoint;
  firstPoint.emplace_back(5);
  firstPoint.push_back(5);
  secondPoint.emplace_back(3);
  secondPoint.push_back(4);
  std::vector<std::vector<float> > result = ob.plan(firstPoint, secondPoint);
  EXPECT_EQ(firstPoint, result.front());
  EXPECT_EQ(firstPoint, result.back());
}

// Tests for class MapManager
TEST(getState, should_pass) {
  MapManager ob = new MapManager("/test.jpg");
  EXPECT_LT(150, ob.getState(0, 0));
}

TEST(testing, TestPass)
{
  std::vector<float> firstPoint;
  firstPoint.emplace_back(2);
  firstPoint.push_back(2);
  MapManager ob = new MapManager("/test.jpg");
  EXPECT_TRUE(!ob.checkObstacle(firstPoint));
}

TEST(testing, TestPass)
{
  std::vector<float> firstPoint;
  firstPoint.emplace_back(2);
  firstPoint.push_back(2);
  MapManager ob = new MapManager("/test.jpg");
  std::vector<std::vector<int> > Cfree = ob.getCfree();
  EXPECT_TRUE(std::find(Cfree.begin(), Cfree.end(), firstPoint) != Cfree.end());
}
