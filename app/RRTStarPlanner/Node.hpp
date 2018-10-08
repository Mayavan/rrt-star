#include <vector>

class Node {
 public:
  std::vector<float> nodePosition;
  float costToCome;
  std::vector<int> branches;
};