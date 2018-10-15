/**
 * @file test.cpp
 * @brief RRT Planner - testing
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

#include <gtest/gtest.h>
#include <vector>

#include "MapManager.hpp"
#include "RRTStarPlanner.hpp"

std::string fileLocation =
    "../DemoFiles/maze.png";

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

TEST(checkObstacle, should_pass) {
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
