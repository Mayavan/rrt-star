#pragma once
#include <vector>

class Node {
 public:
  std::pair<int, int> nodePosition;
  float costToCome;
  std::vector<int> branches;
};
