# RRT-Star
[![Build Status](https://travis-ci.org/Mayavan/rrt-star.svg?branch=master)](https://travis-ci.org/Mayavan/rrt-star)
[![Coverage Status](https://coveralls.io/repos/github/Mayavan/rrt-star/badge.svg?branch=master)](https://coveralls.io/github/Mayavan/rrt-star?branch=master)
---

## Overview

A path planner module to find a feasible path from the given start point to the given end point using RRT*. This planner can be used in the Acme industrial AGVs to quickly find a path from one point to another in a factory workspace while avoiding obstacles. Sampling based algorithm like RRT* give quick solution to using randomized sampling in search space. The module will require an image with black and white pixels (black specifying obstacles and white specifying free space), the scale of pixels in the map image, the clearance distance required for the robot, the minimum number of iterations, the start point and the target point to return a sequence of points as the result of the RRT* planning algorithm.

![Overview](./RRT_planner.png)

## License

The MIT license definition for this project can be viewed [here](https://opensource.org/licenses/MIT)
.

## SIP Process
SIP Process is detailed in [here](https://docs.google.com/spreadsheets/d/1cSA6AFp7Eeqrku6nSDFxkTTb4TWTfvhlCbPjua-hT9A/edit?usp=sharing)

## Installing Dependencies

## Standard install via command-line
```
git clone --recursive https://github.com/Mayavan/rrt-star
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app
```