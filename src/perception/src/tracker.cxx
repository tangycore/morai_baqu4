#include "lidar_clustering/tracker.h"
#include <cmath>
#include <numeric>
#include <algorithm>
#include <ros/ros.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>

Tracker::Tracker()
    : next_id_(0),
      iou_threshold_(0.2),
      distance_threshold_(1.2),
      max_lost_frames_(6),
      history_size_(6),
      min_age_for_lost_track_(3),
      min_score_for_lost_track_(0.6),
      min_size_for_track_(0.03) {}

double Tracker::computeIOU(const vision_msgs::Detection3D& det,
                           const TrackedObject& track)
{
    double det_x = det.bbox.center.position.x;
    double det_y = det.bbox.center.position.y;
    double det_w = det.bbox.size.x;
    double det_h = det.bbox.size.y;

    double det_x1 = det_x - det_w / 2, det_x2 = det_x + det_w / 2;
    double det_y1 = det_y - det_h / 2, det_y2 = det_y + det_h / 2;

    double track_x1 = track.x - track.length / 2, track_x2 = track.x + track.length / 2;
    double track_y1 = track.y - track.width / 2, track_y2 = track.y + track.width / 2;

    double inter_x1 = std::max(det_x1, track_x1);
    double inter_x2 = std::min(det_x2, track_x2);
    double inter_y1 = std::max(det_y1, track_y1);
    double inter_y2 = std::min(det_y2, track_y2);

    if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) return 0.0;

    double inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    double det_area   = det_w * det_h;
    double track_area = track.length * track.width;
    double uni_area   = det_area + track_area - inter_area;

    if (uni_area <= 1e-12) return 0.0;
    return inter_area / uni_area;
}

double Tracker::computeDistance(const vision_msgs::Detection3D& det,
                                const TrackedObject& track)
{
    double dx = det.bbox.center.position.x - track.x;
    double dy = det.bbox.center.position.y - track.y;
    return std::sqrt(dx*dx + dy*dy);
}

void Tracker::update(vision_msgs::Detection3DArray& detections, double timestamp)
{
    int n_dets = static_cast<int>(detections.detections.size());
    int n_trks = static_cast<int>(tracks_.size());

    // IOU + 거리 점수 행렬
    std::vector<std::vector<double>> score(n_trks, std::vector<double>(n_dets, 0.0));
    for (int i = 0; i < n_trks; ++i) {
        for (int j = 0; j < n_dets; ++j) {
            double iou = computeIOU(detections.detections[j], tracks_[i]);
            double dist = computeDistance(detections.detections[j], tracks_[i]);
            score[i][j] = (dist < distance_threshold_)
                        ? iou + (1.0 - dist / distance_threshold_) * 0.5
                        : iou;
        }
    }

    // Greedy 매칭
    std::vector<int> t2d(n_trks, -1), d2t(n_dets, -1);
    std::vector<bool> tm(n_trks, false), dm(n_dets, false);

    while (true) {
        double best = iou_threshold_;
        int bi = -1, bj = -1;
        for (int i = 0; i < n_trks; ++i) {
            if (tm[i]) continue;
            for (int j = 0; j < n_dets; ++j) {
                if (dm[j]) continue;
                if (score[i][j] > best) {
                    best = score[i][j];
                    bi = i; bj = j;
                }
            }
        }
        if (bi < 0) break;
        t2d[bi] = bj; d2t[bj] = bi;
        tm[bi] = true; dm[bj] = true;
    }

    // 기존 트랙 업데이트
    for (int i = 0; i < n_trks; ++i) {
        if (t2d[i] != -1) {
            int j = t2d[i];
            auto& det = detections.detections[j];
            auto& trk = tracks_[i];

            trk.x_history.push_back(trk.x);
            trk.y_history.push_back(trk.y);
            trk.timestamps.push_back(timestamp);
            if ((int)trk.x_history.size() > history_size_) {
                trk.x_history.pop_front();
                trk.y_history.pop_front();
                trk.timestamps.pop_front();
            }

            trk.x = det.bbox.center.position.x;
            trk.y = det.bbox.center.position.y;
            trk.z = det.bbox.center.position.z;
            trk.length = det.bbox.size.x;
            trk.width  = det.bbox.size.y;
            trk.height = det.bbox.size.z;

            if ((int)trk.x_history.size() >= 2) {
                double dt = trk.timestamps.back() - trk.timestamps.front();
                if (dt > 0.01) {
                    trk.vx = (trk.x - trk.x_history.front()) / dt;
                    trk.vy = (trk.y - trk.y_history.front()) / dt;
                }
            }

            trk.age++;
            trk.lost_frames = 0;

            if (det.results.empty()) det.results.resize(1);
            det.results[0].id = trk.id;
        } else {
            auto& trk = tracks_[i];
            if (trk.lost_frames == 0 && (int)trk.x_history.size() >= 2) {
                double dt = 0.1; // 가정 10Hz
                trk.x += trk.vx * dt;
                trk.y += trk.vy * dt;
            }
            trk.lost_frames++;
        }
    }

    // 새 트랙 생성
    for (int j = 0; j < n_dets; ++j) {
        if (d2t[j] != -1) continue;
        auto& det = detections.detections[j];
        double size = det.bbox.size.x * det.bbox.size.y;
        if (size < min_size_for_track_) {
            ROS_DEBUG("Skip small detection: size=%.3f m^2", size);
            continue;
        }
        TrackedObject t;
        t.id = next_id_++;
        t.x = det.bbox.center.position.x;
        t.y = det.bbox.center.position.y;
        t.z = det.bbox.center.position.z;
        t.length = det.bbox.size.x;
        t.width  = det.bbox.size.y;
        t.height = det.bbox.size.z;
        t.age = 1;
        t.lost_frames = 0;

        tracks_.push_back(t);
        if (det.results.empty()) det.results.resize(1);
        det.results[0].id = t.id;
    }

    // Lost 트랙도 Detection으로 발행
    for (const auto& t : tracks_) {
        if (t.lost_frames <= 0 || t.lost_frames > max_lost_frames_) continue;
        double score = 1.0 - (t.lost_frames / (double)max_lost_frames_);
        if (t.age >= min_age_for_lost_track_ && score >= min_score_for_lost_track_) {
            vision_msgs::Detection3D lost;
            lost.bbox.center.position.x = t.x;
            lost.bbox.center.position.y = t.y;
            lost.bbox.center.position.z = t.z;
            lost.bbox.center.orientation.w = 1.0;
            lost.bbox.size.x = t.length;
            lost.bbox.size.y = t.width;
            lost.bbox.size.z = t.height;

            vision_msgs::ObjectHypothesisWithPose res;
            res.id = t.id;
            res.score = score;
            lost.results.push_back(res);
            detections.detections.push_back(lost);
        }
    }

    // 오래된 트랙 제거
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const TrackedObject& t){ return t.lost_frames > max_lost_frames_; }),
        tracks_.end());

    int vis = 0, lost = 0;
    for (const auto& t : tracks_) (t.lost_frames==0 ? ++vis : ++lost);

    ROS_DEBUG("Tracking: %lu total (%d visible, %d lost)",
              tracks_.size(), vis, lost);
}
