#ifndef TRACKER_H
#define TRACKER_H

#include <vector>
#include <deque>
#include <algorithm>

namespace vision_msgs {
struct Detection3D;
struct Detection3DArray;
}

struct TrackedObject {
    int id;
    int age;
    int lost_frames;
    double x, y, z;
    double vx, vy;
    double length, width, height;
    std::deque<double> x_history, y_history, timestamps;

    TrackedObject()
        : id(-1), age(0), lost_frames(0),
          x(0), y(0), z(0), vx(0), vy(0),
          length(0), width(0), height(0) {}
};

class Tracker {
private:
    std::vector<TrackedObject> tracks_;
    int    next_id_;
    double iou_threshold_;
    double distance_threshold_;
    int    max_lost_frames_;
    int    history_size_;
    int    min_age_for_lost_track_;
    double min_score_for_lost_track_;
    double min_size_for_track_;

public:
    Tracker();

    double computeIOU(const vision_msgs::Detection3D& det,
                      const TrackedObject& track);
    double computeDistance(const vision_msgs::Detection3D& det,
                           const TrackedObject& track);
    void   update(vision_msgs::Detection3DArray& detections, double timestamp);

    const std::vector<TrackedObject>& getTracks() const { return tracks_; }
};

#endif // TRACKER_H
