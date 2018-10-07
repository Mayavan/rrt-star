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

## Build

To build the project, in Eclipse, unfold boilerplate-eclipse project in Project Explorer,
unfold Build Targets, double click on "all" to build all projects.

## Run

1. In Eclipse, right click on the boilerplate-eclipse in Project Explorer,
select Run As -> Local C/C++ Application

2. Choose the binaries to run (e.g. shell-app, cpp-test for unit testing)


## Debug


1. Set breakpoint in source file (i.e. double click in the left margin on the line you want 
the program to break).

2. In Eclipse, right click on the boilerplate-eclipse in Project Explorer, select Debug As -> 
Local C/C++ Application, choose the binaries to run (e.g. shell-app).

3. If prompt to "Confirm Perspective Switch", select yes.

4. Program will break at the breakpoint you set.

5. Press Step Into (F5), Step Over (F6), Step Return (F7) to step/debug your program.

6. Right click on the variable in editor to add watch expression to watch the variable in 
debugger window.

7. Press Terminate icon to terminate debugging and press C/C++ icon to switch back to C/C++ 
perspetive view (or Windows->Perspective->Open Perspective->C/C++).


## Plugins

- CppChEclipse

    To install and run cppcheck in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> cppcheclipse.
    Set cppcheck binary path to "/usr/bin/cppcheck".

    2. To run CPPCheck on a project, right click on the project name in the Project Explorer 
    and choose cppcheck -> Run cppcheck.


- Google C++ Sytle

    To include and use Google C++ Style formatter in Eclipse

    1. In Eclipse, go to Window -> Preferences -> C/C++ -> Code Style -> Formatter. 
    Import [eclipse-cpp-google-style][reference-id-for-eclipse-cpp-google-style] and apply.

    2. To use Google C++ style formatter, right click on the source code or folder in 
    Project Explorer and choose Source -> Format

[reference-id-for-eclipse-cpp-google-style]: https://raw.githubusercontent.com/google/styleguide/gh-pages/eclipse-cpp-google-style.xml

- Git

    It is possible to manage version control through Eclipse and the git plugin, but it typically requires creating another project. If you're interested in this, try it out yourself and contact me on Canvas.
