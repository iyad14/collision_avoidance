#pragma once
#include <vector>

struct flatscanMetadata {
  float min_angle;
  float max_angle;
  float range_min;
  float range_max;
  float increment_angle;
  int beam_count;
};

struct flatscan3DOAModel {
  std::vector<double> ranges;
  std::vector<double> angles;
  std::vector<float> heights;
  std::vector<bool> hanging;
  std::vector<bool> state;
  std::vector<bool> visible;
};
