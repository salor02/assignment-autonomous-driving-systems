#ifndef TRACKLET_H_
#define TRACKLET_H_

#include <vector>
#include <cmath>

#include "KalmanFilter.h"

class Tracklet
{
public:
  Tracklet(int idTrack, double x, double y);
  ~Tracklet();

  void predict();
  void update(double x, double y, bool lidarStatus);

  // getters
  double getX() { return kf_.getX(); }
  double getY() { return kf_.getY(); }
  double getXCovariance() { return kf_.getXCovariance(); }
  double getYCovariance() { return kf_.getYCovariance(); }
  double getVelX() { return kf_.getVelX(); }
  double getVelY() { return kf_.getVelY(); }
  int getLossCount() { return loss_count_; }
  int getId() { return id_; }
  double getLength() { return length_; }

private:
  // filter
  KalmanFilter kf_;

  // tracklet id
  int id_;

  // number of loss since last update
  int loss_count_;

  // tracklet length
  double length_;
  void lengthUpdate(double x0, double y0, double x1, double y1);
};

#endif // TRACKLET_H_
