#include <algorithm>
#include <tuple>
#include <vector>

#include "Eigen/Dense"
#include "models/Hungarian.h"
#include "tuple"
#include "types.h"

using namespace std;

namespace kf {
namespace util {

inline int safeSizeTToInt(std::size_t size) {
  if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    throw std::overflow_error("Conversion would result in overflow");
  }
  return static_cast<int>(size);
}

inline vector<vector<double>> cDist(const Matrix<Eigen::Dynamic, Eigen::Dynamic>& X,
    const Matrix<Eigen::Dynamic, Eigen::Dynamic>& Y) {
  // const int N = X.rows();
  // const int K = Y.rows();

  // // Allocate parts of the expression
  // Matrix<Eigen::Dynamic, Eigen::Dynamic> XX, YY, XY;

  // XX.resize(N, 1);
  // YY.resize(1, K);
  // XY.resize(N, K);

  // // Compute norms
  // XX = X.array().square().rowwise().sum();
  // YY = Y.array().square().rowwise().sum().transpose();
  // XY = 2 * X * Y.transpose();

  // Matrix<Eigen::Dynamic, Eigen::Dynamic> D;
  // D.resize(N, K);

  // // Compute final expression
  // D = XX * Eigen::MatrixXd::Ones(1, K);
  // D = D + Eigen::MatrixXd::Ones(N, 1) * YY;
  // D = D - XY;

  // D = D.array().sqrt();

  // vector<vector<double>> costMatrix(D.rows(), vector<double>(D.cols(), 1));
  // for (std::size_t i = 0; i < costMatrix.size(); i++) {
  //   for (std::size_t j = 0; j < costMatrix[i].size(); j++) {
  //     costMatrix[i][j] = D(i, j);
  //   }
  // }

  // More optimal version is to transpose D and iterate with one for loop to populate the cost
  // matrix.

  const int N = Y.rows();
  const int K = X.rows();

  // Allocate parts of the expression
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> XX, YY, XY;

  XX.resize(N, 1);
  YY.resize(1, K);
  XY.resize(N, K);

  // Compute norms
  XX = Y.array().square().rowwise().sum();
  YY = X.array().square().rowwise().sum().transpose();
  XY = 2 * Y * X.transpose();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D, DT;
  D.resize(N, K);

  // Compute final expression
  D = XX * Eigen::MatrixXd::Ones(1, K);
  D = D + Eigen::MatrixXd::Ones(N, 1) * YY;
  D = D - XY;

  D = D.array().sqrt();

  vector<vector<double>> costMatrix;
  std::vector<double> col;
  for (unsigned int i = 0; i < D.cols(); i++) {
    col.assign(D.col(i).data(), D.col(i).data() + D.col(i).size());
    costMatrix.push_back(col);
  }

  return costMatrix;
}

inline std::tuple<vector<tuple<int, int>>, vector<int>, vector<int>> assignTracks2Detections(
    const Matrix<Eigen::Dynamic, Eigen::Dynamic>& centroidTracks,
    const Matrix<Eigen::Dynamic, Eigen::Dynamic>& centroidDetections, double distanceThreshold) {
  if ((centroidTracks.rows() == 0) || (centroidDetections.rows() == 0)) {
    int n_rows = centroidTracks.rows() != 0 ? centroidTracks.rows() : centroidDetections.rows();
    std::vector<int> ret_unmatched;

    for (int i = 0; i < n_rows; i++) {
      ret_unmatched.push_back(i);
    }

    return std::make_tuple(std::vector<std::tuple<int, int>>(), ret_unmatched, std::vector<int>());
  }

  vector<int> assigned_tracks, assigned_detections, unmatched_detections, unmatched_tracks;
  vector<tuple<int, int>> matches;

  vector<vector<double>> costMatrix = cDist(centroidTracks, centroidDetections);

  HungarianAlgorithm HungAlgo;
  vector<int> assignment;

  HungAlgo.Solve(costMatrix, assignment);

  // std::cout << "Assignment: " << std::endl;
  // for (std::size_t i = 0; i < assignment.size(); ++i) {
  //   std::cout << " " << assignment[i] << " ";
  // }
  // std::cout << std::endl;

  for (std::size_t x = 0; x < assignment.size(); x++) {
    if (assignment[x] == -1) {
      unmatched_tracks.push_back(x);
      continue;
    } else {
      assigned_tracks.push_back(x);
      assigned_detections.push_back(assignment[x]);
    }
  }

  for (int d = 0; d < centroidDetections.rows(); d++) {
    if (std::find(assigned_detections.begin(), assigned_detections.end(), d) ==
        assigned_detections.end()) {
      unmatched_detections.push_back(d);
    }
  }

  // for (int t = 0; t < centroidTracks.rows(); t++) {
  //   if (std::find(assigned_tracks.begin(), assigned_tracks.end(), t) == assigned_tracks.end()) {
  //     unmatched_tracks.push_back(t);
  //   }
  // }

  for (std::size_t i = 0; i < assigned_detections.size(); i++) {
    int t = assigned_tracks[i];
    int d = assigned_detections[i];

    if (costMatrix[t][d] > distanceThreshold) {
      unmatched_detections.push_back(d);
      unmatched_tracks.push_back(t);
    } else {
      matches.push_back(std::make_tuple(t, d));
    }
  }

  return std::make_tuple(matches, unmatched_detections, unmatched_tracks);
}

}  // namespace util
}  // namespace kf