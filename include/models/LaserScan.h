#pragma once
#include <vector>

struct LaserScan {
  float min_angle;
  float max_angle;
  float range_min;
  float range_max;
  float increment_angle;
  
  std::vector<float> ranges;       
  std::vector<float> intensities;
};