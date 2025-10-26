#ifndef KALMAN_IOU_TRACKER_H
#define KALMAN_IOU_TRACKER_H

#include <vector>
#include <deque>
#include <cmath>
#include <ros/ros.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>
#include <opencv2/video/tracking.hpp>

/**
 * Improved Tracker: IOU Matching + Kalman Filter
 * 
 * 장점:
 * 1. IOU 기반 매칭 (robust data association)
 * 2. Kalman Filter로 노이즈 제거
 * 3. 동적 객체 수 지원 (N개 가능)
 */

struct KalmanTrackedObject
{
    int id;
    int age;
    int lost_frames;
    
    // Kalman Filter
    cv::KalmanFilter kf;
    
    // State: [x, y, vx, vy]
    double x, y, z;
    double vx, vy;
    double length, width, height;
    
    // Predicted state for IOU matching
    double pred_x, pred_y;
    
    KalmanTrackedObject() : id(-1), age(0), lost_frames(0),
                            x(0), y(0), z(0), vx(0), vy(0),
                            length(0), width(0), height(0),
                            pred_x(0), pred_y(0)
    {
        // Initialize Kalman Filter
        // State: [x, y, vx, vy], Measurement: [x, y]
        kf = cv::KalmanFilter(4, 2, 0, CV_32F);
        
        // Transition Matrix: x' = x + vx*dt, y' = y + vy*dt
        kf.transitionMatrix = (cv::Mat_<float>(4, 4) << 
            1, 0, 1, 0,   // x' = x + vx
            0, 1, 0, 1,   // y' = y + vy
            0, 0, 1, 0,   // vx' = vx
            0, 0, 0, 1);  // vy' = vy
        
        // Measurement Matrix: measure only [x, y]
        cv::setIdentity(kf.measurementMatrix);
        
        // Process Noise
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
        
        // Measurement Noise
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1));
        
        // Error Covariance
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));
    }
    
    void initKalman(double init_x, double init_y)
    {
        kf.statePost.at<float>(0) = init_x;  // x
        kf.statePost.at<float>(1) = init_y;  // y
        kf.statePost.at<float>(2) = 0;       // vx
        kf.statePost.at<float>(3) = 0;       // vy
    }
    
    void predict()
    {
        cv::Mat prediction = kf.predict();
        pred_x = prediction.at<float>(0);
        pred_y = prediction.at<float>(1);
        vx = prediction.at<float>(2);
        vy = prediction.at<float>(3);
    }
    
    void update(double meas_x, double meas_y)
    {
        cv::Mat_<float> measurement(2, 1);
        measurement(0) = meas_x;
        measurement(1) = meas_y;
        
        cv::Mat estimated = kf.correct(measurement);
        
        x = estimated.at<float>(0);
        y = estimated.at<float>(1);
        vx = estimated.at<float>(2);
        vy = estimated.at<float>(3);
    }
};

class KalmanIOUTracker
{
private:
    std::vector<KalmanTrackedObject> tracks_;
    int next_id_;
    
    double iou_threshold_;
    int max_lost_frames_;
    
public:
    KalmanIOUTracker() : next_id_(0), iou_threshold_(0.3), max_lost_frames_(5) {}
    
    // IOU 계산 (예측 위치 사용)
    double computeIOU(const vision_msgs::Detection3D& det, const KalmanTrackedObject& track)
    {
        double det_x = det.bbox.center.position.x;
        double det_y = det.bbox.center.position.y;
        double det_w = det.bbox.size.x;
        double det_h = det.bbox.size.y;
        
        // Use predicted position from Kalman Filter
        double track_x = track.pred_x;
        double track_y = track.pred_y;
        
        // Bounding box corners
        double det_x1 = det_x - det_w / 2;
        double det_x2 = det_x + det_w / 2;
        double det_y1 = det_y - det_h / 2;
        double det_y2 = det_y + det_h / 2;
        
        double track_x1 = track_x - track.length / 2;
        double track_x2 = track_x + track.length / 2;
        double track_y1 = track_y - track.width / 2;
        double track_y2 = track_y + track.width / 2;
        
        // Intersection
        double inter_x1 = std::max(det_x1, track_x1);
        double inter_x2 = std::min(det_x2, track_x2);
        double inter_y1 = std::max(det_y1, track_y1);
        double inter_y2 = std::min(det_y2, track_y2);
        
        if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1)
            return 0.0;
        
        double inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
        double det_area = det_w * det_h;
        double track_area = track.length * track.width;
        double union_area = det_area + track_area - inter_area;
        
        return inter_area / union_area;
    }
    
    void update(vision_msgs::Detection3DArray& detections, double timestamp)
    {
        // 1. Predict all tracks
        for (auto& track : tracks_)
        {
            track.predict();
        }
        
        int n_detections = detections.detections.size();
        int n_tracks = tracks_.size();
        
        // 2. IOU 행렬 계산 (predicted position 사용)
        std::vector<std::vector<double>> iou_matrix(n_tracks, std::vector<double>(n_detections, 0.0));
        for (int i = 0; i < n_tracks; i++)
        {
            for (int j = 0; j < n_detections; j++)
            {
                iou_matrix[i][j] = computeIOU(detections.detections[j], tracks_[i]);
            }
        }
        
        // 3. Greedy 매칭
        std::vector<int> track_to_detection(n_tracks, -1);
        std::vector<int> detection_to_track(n_detections, -1);
        std::vector<bool> matched_tracks(n_tracks, false);
        std::vector<bool> matched_detections(n_detections, false);
        
        while (true)
        {
            double max_iou = iou_threshold_;
            int best_track = -1;
            int best_detection = -1;
            
            for (int i = 0; i < n_tracks; i++)
            {
                if (matched_tracks[i]) continue;
                for (int j = 0; j < n_detections; j++)
                {
                    if (matched_detections[j]) continue;
                    if (iou_matrix[i][j] > max_iou)
                    {
                        max_iou = iou_matrix[i][j];
                        best_track = i;
                        best_detection = j;
                    }
                }
            }
            
            if (best_track == -1) break;
            
            track_to_detection[best_track] = best_detection;
            detection_to_track[best_detection] = best_track;
            matched_tracks[best_track] = true;
            matched_detections[best_detection] = true;
        }
        
        // 4. 기존 트랙 업데이트 (Kalman Correct)
        for (int i = 0; i < n_tracks; i++)
        {
            if (track_to_detection[i] != -1)
            {
                int det_idx = track_to_detection[i];
                auto& det = detections.detections[det_idx];
                auto& track = tracks_[i];
                
                // Kalman Filter Update
                track.update(det.bbox.center.position.x, det.bbox.center.position.y);
                
                // Update other info
                track.z = det.bbox.center.position.z;
                track.length = det.bbox.size.x;
                track.width = det.bbox.size.y;
                track.height = det.bbox.size.z;
                
                track.age++;
                track.lost_frames = 0;
                
                det.results[0].id = track.id;
            }
            else
            {
                tracks_[i].lost_frames++;
            }
        }
        
        // 5. 새로운 트랙 생성
        for (int j = 0; j < n_detections; j++)
        {
            if (detection_to_track[j] == -1)
            {
                KalmanTrackedObject new_track;
                new_track.id = next_id_++;
                new_track.x = detections.detections[j].bbox.center.position.x;
                new_track.y = detections.detections[j].bbox.center.position.y;
                new_track.z = detections.detections[j].bbox.center.position.z;
                new_track.length = detections.detections[j].bbox.size.x;
                new_track.width = detections.detections[j].bbox.size.y;
                new_track.height = detections.detections[j].bbox.size.z;
                new_track.age = 1;
                new_track.lost_frames = 0;
                
                // Initialize Kalman Filter
                new_track.initKalman(new_track.x, new_track.y);
                
                tracks_.push_back(new_track);
                detections.detections[j].results[0].id = new_track.id;
            }
        }
        
        // 6. 오래된 트랙 제거
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                          [this](const KalmanTrackedObject& t) { 
                              return t.lost_frames > max_lost_frames_; 
                          }),
            tracks_.end()
        );
        
        ROS_INFO("KalmanIOU Tracking: %lu active tracks", tracks_.size());
    }
    
    const std::vector<KalmanTrackedObject>& getTracks() const { return tracks_; }
};

#endif // KALMAN_IOU_TRACKER_H