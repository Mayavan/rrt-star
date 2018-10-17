/**
 * @file RRTStarPlannerTest.cpp
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

#include "RRTStarPlanner.hpp"

std::string fileLocation =
    "../DemoFiles/maze.png";

int stepSize = 5;
int64 minimumIteration = 10000;

// Tests for class RRTStarPlanner
TEST(calculateDistance, should_pass) {
  RRTStarPlanner ob(fileLocation, stepSize, minimumIteration);
  std::pair<int, int> firstPoint(0, 0);
  std::pair<int, int> secondPoint(3, 4);
  EXPECT_EQ(5, ob.calculateDistance(firstPoint, secondPoint));
}

TEST(plan, should_pass) {
  RRTStarPlanner ob(fileLocation, stepSize, minimumIteration);
  std::pair<int, int> firstPoint(25, 25);
  std::pair<int, int> secondPoint(25, 375);
  std::vector<std::pair<int, int> > result = ob.plan(firstPoint, secondPoint);
  EXPECT_EQ(secondPoint, result.front());
  EXPECT_EQ(firstPoint, result.back());
}

TEST(testTargetInObstacle, should_pass) {
  RRTStarPlanner ob(fileLocation, stepSize, minimumIteration);
  std::pair<int, int> firstPoint(1, 1);
  std::pair<int, int> secondPoint(25, 375);
  std::vector<std::pair<int, int> > result = ob.plan(firstPoint, secondPoint);
  EXPECT_TRUE(result.empty());
}

TEST(plotPlan, should_pass) {
  RRTStarPlanner ob(fileLocation, stepSize, minimumIteration);
  std::pair<int, int> firstPoint(25, 25);
  std::pair<int, int> secondPoint(25, 375);
  std::vector<std::pair<int, int> > result = ob.plan(firstPoint, secondPoint);
  EXPECT_EQ(0, ob.plotPlan(result));
}
