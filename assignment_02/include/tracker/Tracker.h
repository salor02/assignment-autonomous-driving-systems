#ifndef TRACKER_H_
#define TRACKER_H_

#include "tracker/Tracklet.h"
#include "viewer/Renderer.h"
#include <limits>

class Tracker
{
public:
  Tracker();
  ~Tracker();

  // handle tracklets
  void removeTracks();
  void addTracks(const std::vector<bool> &associated_detections,
                 const std::vector<double> &centroids_x,
                 const std::vector<double> &centroids_y);

  // associate tracklets and detections
  void dataAssociation(std::vector<bool> &associated_detections,
                       const std::vector<double> &centroids_x,
                       const std::vector<double> &centroids_y);

  // track objects
  void track(const std::vector<double> &centroids_x,
             const std::vector<double> &centroids_y,
             bool lidarStatus);

  // getters
  const std::vector<Tracklet> &getTracks() { return tracks_; }
  std::pair<int, double> getLongestTracklet() {return longest_path_; }
  viewer::Box getTrackedArea() {return area_; }
  int getAreaCount();
  std::pair<int, int> getLongestTrackInArea();

private:
  // tracklets
  std::vector<Tracklet> tracks_;
  int cur_id_;

  // association
  std::vector<std::pair<int, int>> associated_track_det_ids_;

  // thresholds
  double distance_threshold_;
  double covariance_threshold;
  int loss_threshold;

  //memorize the id and length of longest path
  std::pair<int, double> longest_path_;

  //useful for counting people entering in a defined area
  viewer::Box area_;
  std::vector<int> area_tracks_frames_;
};

#endif // TRACKER_H_
