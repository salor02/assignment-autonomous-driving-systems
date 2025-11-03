#include "tracker/Tracklet.h"

Tracklet::Tracklet(int idTrack, double x, double y)
{
  // set id
  id_ = idTrack;

  // initialize filter
  kf_.init(0.1);
  kf_.setState(x, y);

  // set loss count to 0
  loss_count_ = 0;

  // set length to 0
  length_ = 0;
}

Tracklet::~Tracklet()
{
}

// Predict a single measurement
void Tracklet::predict()
{
  kf_.predict();
  loss_count_++;
}

// Update with a real measurement
void Tracklet::update(double x, double y, bool lidarStatus)
{
  Eigen::VectorXd raw_measurements_ = Eigen::VectorXd(2);

  //get coords before update
  double x0 = kf_.getX();
  double y0 = kf_.getY();
  
  // measurement update
  if (lidarStatus)
  {
    raw_measurements_ << x, y;

    kf_.update(raw_measurements_);
    loss_count_ = 0;

    //get coords after update
    double x1 = kf_.getX();
    double y1 = kf_.getY();
    
    /*  total path length is updated computing euclidean distance between
        the coords before and after the update  */
    lengthUpdate(x0, y0, x1, y1);
  }
}

void Tracklet::lengthUpdate(double x0, double y0, double x1, double y1){
  double step_length = sqrt(pow(x0-x1, 2) + pow(y0-y1, 2));
  length_ += step_length;
}

