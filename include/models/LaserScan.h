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
  std::vector<float> heights;
  std::vector<bool> hanging;
  std::vector<bool> states;
  std::vector<bool> in_fov;
  std::vector<bool> camera_only_detections;
};