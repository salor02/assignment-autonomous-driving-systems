#include "tracker/Tracker.h"
#include <iostream>

#define EUCLIDEAN_DISTANCE

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 1.0; // meters
    covariance_threshold = 5.0; 
    loss_threshold = 30; //number of frames the track has not been seen

    // area size definition. Clusters that enter this area will be counted.
    area_.x_min=-1.5;
    area_.y_min=-1.5;
    area_.z_min=0;
    area_.x_max=1.5;
    area_.y_max=1.5;
    area_.z_max=0;
}
Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;

    /*
        A trakclet is deleted from the list if it has not been associated for a subsequent number of frames greater than loss_threshold or
        if its covariance on one of the axis is greater than covariance_threshold.
        This mechanism translates in: "if a tracklet has not been associated for a sufficient amount of frames then it can be discarded, or
        if its state is too uncertain then it probably doesn't represent the correct reality anymore"
    */
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        if(tracks_[i].getLossCount() > loss_threshold)
            continue;
        if(tracks_[i].getXCovariance() > covariance_threshold || tracks_[i].getYCovariance() > covariance_threshold){
            // std::cerr<<"[" + std::to_string(tracks_[i].getId()) + "]" + std::to_string(tracks_[i].getXCovariance()) + " " + std::to_string(tracks_[i].getYCovariance()) +";"<<std::endl;
            continue;
        }
        tracks_to_keep.push_back(tracks_[i]);
    }

    tracks_.swap(tracks_to_keep);
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections (meaning that the detection is new and represents its own tracklet)
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i]){
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
            // each time a new tracklet is created a new counter it is initialized in order to count the number of frames this tracklet will be within the area
            area_tracks_frames_.push_back(0);
        }
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Remind this vector contains a pair of tracks and its corresponding
    associated_track_det_ids_.clear();

    /*
        The following lines do:
            - compute the closest detection wrt considered tracklet
            - if the closest detection has not been associated yet and its distance from the considered track is less than distance_threshold then
              associate the detection to that tracklet
        Keep in mind that if a detection is not associated at this step it's going to represent a new tracklet 
    */
    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            // Logic to find the closest detection (centroids_x,centroids_y) to the current track (tracks_) 
            #ifdef EUCLIDEAN_DISTANCE
            double dist = sqrt(pow((centroids_x[j] - tracks_[i].getX()),2) + pow((centroids_y[j] - tracks_[i].getY()),2));
            if(dist < min_dist){
                min_dist = dist;
                closest_point_id = j;
            }
            #endif
        }

        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}


/*  A tracklet is considered to be "entered in the area" if the number of frames it spends within the area is greater than zero */
int Tracker::getAreaCount(){
    return area_tracks_frames_.size() - std::count(area_tracks_frames_.begin(), area_tracks_frames_.end(), 0);
}

/*  Gets the tracklet which spent most time in the area */
std::pair<int,int> Tracker::getLongestTrackInArea(){
    auto max_frames_it = std::max_element(area_tracks_frames_.begin(), area_tracks_frames_.end());
    int track_id = std::distance(area_tracks_frames_.begin(), max_frames_it);
    int max_value = *max_frames_it;

    return std::pair<int,int>(track_id, max_value);
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{

    std::vector<bool> associated_detections(centroids_x.size(), false);

    // For each track --> Predict the position of the tracklets
    for (size_t i = 0; i < tracks_.size(); ++i){
        tracks_[i].predict();
        std::cerr<<"[" + std::to_string(tracks_[i].getId()) + "]" + std::to_string(tracks_[i].getYaw()) + " " + std::to_string(tracks_[i].getVelY()) +";";
    }
    std::cerr<<std::endl;
    
    // Associate the predictions with the detections
    dataAssociation(associated_detections, centroids_x, centroids_y);

    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);

        // updates longest tracklet
        if(tracks_[track_id].getLength() > longest_path_.second){
            longest_path_.first = tracks_[track_id].getId();
            longest_path_.second = tracks_[track_id].getLength();
        }

        // records a new frame if the tracklet is currently within the area
        if(tracks_[track_id].getX() >= area_.x_min && tracks_[track_id].getX() <= area_.x_max &&
            tracks_[track_id].getY() >= area_.y_min && tracks_[track_id].getY() <= area_.y_max){
                area_tracks_frames_[tracks_[track_id].getId()] += 1;
        }
    }

    // Remove dead tracklets
    removeTracks();

    // Add new tracklets
    addTracks(associated_detections, centroids_x, centroids_y);
}
