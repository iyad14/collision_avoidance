#include <stdint.h>

#include "Eigen/Dense"

namespace kf {

template <int ROW, int COL>
using Matrix = Eigen::Matrix<double, ROW, COL>;

template <int ROW>
using Vector = Eigen::Matrix<double, ROW, 1>;

template <int ROW>
using Vectori = Eigen::Matrix<int, ROW, 1>;
}  // namespace kf