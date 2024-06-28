#include <iostream>
#include <map>
#include <vector>

#include "KalmanFilter.h"
#include "types.h"
#include "util.h"

namespace kf {

/**
 * Class that describes a Tracked object and that allows
 * to perform predictions on the next state of that object.
 */
class Track {
 private:
  int _trackId;
  int _skippedFrames = 0;
  std::string _label = "dynamic";
  Vector<2> _observation;

 public:
  int& trackId() { return _trackId; }
  const int& trackId() const { return _trackId; }

  int& skippedFrames() { return _skippedFrames; }
  const int& skippedFrames() const { return _skippedFrames; }

  std::string& label() { return _label; }
  const std::string& label() const { return _label; }

  Vector<2>& observation() { return _observation; }
  const Vector<2>& observation() const { return _observation; }

  KalmanFilter filter;

  Track(const Vector<2>& detection, int id, float dt, float initCov, Vector<2> paramsCov)
      : filter(dt, initCov, paramsCov) {
    filter.correct(detection);
    _trackId = id;
    _observation = detection;
  }

  Vector<4> predict() { return filter.predict(); }

  void update(const Vector<2>& observation) {
    _observation = observation;
    filter.correct(observation);
  }
};

/**
 * Class that Matches the tracks (tn-1) to detections (tn)
 * and handles both detections and tracks with no match.
 * Additionally uses KalmanFilter to have predctions on the velocities of the matched objects.
 * Thresholds the velocities to extract a classification on the object state (static/dynamic).
 */
class MOKFTracker {
 private:
  int trackId = 0;
  float dt = 1.0;
  int maxFrameSkipped = 0;
  float centroidThreshold = 0.2;
  float initCov = 100.0;
  Vector<2> paramsCov{kf::Vector<2>::Ones() * 0.1};
  float speedThreshold = 0.1;

  std::map<int, Track> tracks;

 public:
  MOKFTracker(){};
  MOKFTracker(float dt, int maxFrameSkipped, float centroidThreshold, float initCov,
      Vector<2> paramsCov, float speedThreshold) {
    this->maxFrameSkipped = maxFrameSkipped;
    this->centroidThreshold = centroidThreshold;
    this->dt = dt;
    this->initCov = initCov;
    this->paramsCov = paramsCov;
    this->speedThreshold = speedThreshold;
  }

  void addTrack(Vector<2>& obs, int trackId, std::string label) {
    Track track(obs, trackId, this->dt, this->initCov, this->paramsCov);
    track.label() = label;
    this->tracks.insert({trackId, track});
    this->trackId += 1;
    if (this->trackId == 100) this->trackId = 0;
  }
  void updateTrack(Vector<2>& obs, int trackId, int skippedFrames = 0) {
    Track& currTrack = this->tracks.find(trackId)->second;
    currTrack.skippedFrames() += skippedFrames;
    currTrack.update(obs);

    Vector<4> state = currTrack.filter.vecX();  // X, Y, X', Y'
    double centroidSpeedX = state[2];
    double centroidSpeedY = state[3];

    // Velocity thresholding.
    if ((fabs(centroidSpeedX) <= this->speedThreshold) &&
        (fabs(centroidSpeedY) <= this->speedThreshold)) {
      currTrack.label() = "static";
    } else {
      currTrack.label() = "dynamic";
    }
  }
  void update(const Matrix<Eigen::Dynamic, 2>& mObs, std::vector<bool>& result) {
    const int nObj = mObs.rows();    // detections
    const int obsDim = mObs.cols();  // number of detections

    Matrix<Eigen::Dynamic, Eigen::Dynamic> bboxTracks;

    std::vector<int> trackIds;
    for (const auto& item : this->tracks) {
      trackIds.push_back(item.first);
    }

    int nTracks = util::safeSizeTToInt(trackIds.size());

    // Refresh old tracks.
    if (nTracks != 0) {
      bboxTracks.resize(nTracks, obsDim);
      for (int i = 0; i < nTracks; i++) {
        Vector<2> centroidState =
            this->tracks.find(trackIds[i])->second.predict().head(2);  // centroid tracking 2D ->
                                                                       // state vector dim = 2.
        bboxTracks.row(i) = centroidState;
      }
    }

    // Erase old out of fov tracks.
    if (nObj == 0) {
      for (int i = 0; i < nTracks; i++) {
        int id = trackIds[i];
        this->tracks.find(id)->second.skippedFrames() += 1;
        if (this->tracks.find(id)->second.skippedFrames() > this->maxFrameSkipped) {
          this->tracks.erase(id);
        }
      }
      return;
    }
    // Associate tracks and detections.
    else {
      std::vector<int> unmatchedDetections, unmatchedTracks;
      std::vector<std::tuple<int, int>> matches;

      std::tie(matches, unmatchedDetections, unmatchedTracks) =
          util::assignTracks2Detections(bboxTracks, mObs, this->centroidThreshold);

      // Matches is what we care about.
      for (std::size_t i = 0; i < matches.size(); i++) {
        int t = std::get<0>(matches[i]);
        int d = std::get<1>(matches[i]);

        int trackID = trackIds[t];
        Vector<2> obs = mObs.row(d);
        // std::cout << "Matched:     " << obs << "   with   "
        //           << this->tracks.find(trackIds[i])->second.observation() << std::endl;
        this->updateTrack(obs, trackID);
      }

      // Create new tracks for new detections.
      for (std::size_t i = 0; i < unmatchedDetections.size(); i++) {
        int d = unmatchedDetections[i];
        int trackID = this->trackId;
        Vector<2> obs = mObs.row(d);

        this->addTrack(obs, trackID, "dynamic");
      }

      // Erase newly out of fov tracks.
      for (std::size_t i = 0; i < unmatchedTracks.size(); i++) {
        int t = unmatchedTracks[i];
        int trackID = trackIds[t];
        Vector<2> obs = bboxTracks.row(t);

        this->updateTrack(obs, trackID, 1);
        if (this->tracks.find(trackID)->second.skippedFrames() > this->maxFrameSkipped) {
          this->tracks.erase(trackID);
        }
      }
    }

    // Return array of result that describes the state classification (static is true, dynamic is
    // false).
    for (auto track : this->tracks) {
      // std::cout << "Size of result:  " << result.size() << std::endl;
      // std::cout << "id: " << track.first << ", " << "state: " << track.second.label() << endl
      //           << track.second.observation() << std::endl
      //           << "Velocity:  " << track.second.filter.vecX() << endl;
      for (int i = 0; i < nObj; ++i) {
        if ((track.second.observation()[0] == mObs.row(i)[0]) &&
            (track.second.observation()[1] == mObs.row(i)[1])) {
          if (track.second.label() == "static") {
            result[i] = true;
          } else {
            result[i] = false;
          }
        }
      }
    }
  }
};
}  // namespace kf