#ifndef SIMPLE_TRACKER_H
#define SIMPLE_TRACKER_H

#include <vector>
#include <deque>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <vision_msgs/Detection3D.h>
#include <vision_msgs/Detection3DArray.h>

struct TrackedObject
{
    int id;
    int age;  // 몇 프레임 동안 존재했는지
    int lost_frames;  // 몇 프레임 동안 안 보였는지
    
    // 현재 상태
    double x, y, z;
    double vx, vy;  // 속도
    double length, width, height;
    
    // 이전 위치 (속도 계산용)
    std::deque<double> x_history;
    std::deque<double> y_history;
    std::deque<double> timestamps;
    
    TrackedObject() : id(-1), age(0), lost_frames(0), 
                      x(0), y(0), z(0), vx(0), vy(0),
                      length(0), width(0), height(0) {}
};

class SimpleTracker
{
private:
    std::vector<TrackedObject> tracks_;
    int next_id_;
    
    // Parameters
    double iou_threshold_;          // IOU 임계값
    double distance_threshold_;     // 거리 임계값 (작은 물체용)
    int max_lost_frames_;           // 몇 프레임 안 보이면 삭제할지
    int history_size_;              // 속도 계산용 히스토리 크기
    
    // 노이즈 필터링 파라미터
    int min_age_for_lost_track_;       // Lost Track 발행 최소 나이
    double min_score_for_lost_track_;  // Lost Track 발행 최소 신뢰도
    double min_size_for_track_;        // Track 생성 최소 크기 (m²)
    
public:
    SimpleTracker() 
        : next_id_(0), 
          iou_threshold_(0.2),              // 작은 물체 고려해서 낮춤
          distance_threshold_(2.0),         // 2m 이내면 같은 물체
          max_lost_frames_(8),              // 8프레임 (약 0.8초) 동안 유지
          history_size_(8),
          min_age_for_lost_track_(1),       // 최소 3프레임 이상 살아야 Lost Track 발행
          min_score_for_lost_track_(0.6),   // 신뢰도 60% 이상만 발행
          min_size_for_track_(0.05) {}      // 0.15m² 이상만 Track 생성
    
    // IOU 계산 (2D bounding box)
    double computeIOU(const vision_msgs::Detection3D& det, const TrackedObject& track)
    {
        double det_x = det.bbox.center.position.x;
        double det_y = det.bbox.center.position.y;
        double det_w = det.bbox.size.x;
        double det_h = det.bbox.size.y;
        
        // Bounding box corners
        double det_x1 = det_x - det_w / 2;
        double det_x2 = det_x + det_w / 2;
        double det_y1 = det_y - det_h / 2;
        double det_y2 = det_y + det_h / 2;
        
        double track_x1 = track.x - track.length / 2;
        double track_x2 = track.x + track.length / 2;
        double track_y1 = track.y - track.width / 2;
        double track_y2 = track.y + track.width / 2;
        
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
    
    // 거리 기반 매칭 (작은 물체용)
    double computeDistance(const vision_msgs::Detection3D& det, const TrackedObject& track)
    {
        double dx = det.bbox.center.position.x - track.x;
        double dy = det.bbox.center.position.y - track.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    // Detection과 Track 매칭
    void update(vision_msgs::Detection3DArray& detections, double timestamp)
    {
        int n_detections = detections.detections.size();
        int n_tracks = tracks_.size();
        
        // IOU + 거리 혼합 점수 계산
        std::vector<std::vector<double>> score_matrix(n_tracks, std::vector<double>(n_detections, 0.0));
        for (int i = 0; i < n_tracks; i++)
        {
            for (int j = 0; j < n_detections; j++)
            {
                double iou = computeIOU(detections.detections[j], tracks_[i]);
                double dist = computeDistance(detections.detections[j], tracks_[i]);
                
                // 거리가 임계값 이내면 IOU 점수 부스트
                if (dist < distance_threshold_)
                {
                    score_matrix[i][j] = iou + (1.0 - dist / distance_threshold_) * 0.5;
                }
                else
                {
                    score_matrix[i][j] = iou;
                }
            }
        }
        
        // 간단한 Greedy 매칭
        std::vector<int> track_to_detection(n_tracks, -1);
        std::vector<int> detection_to_track(n_detections, -1);
        std::vector<bool> matched_tracks(n_tracks, false);
        std::vector<bool> matched_detections(n_detections, false);
        
        // 점수가 가장 높은 순으로 매칭
        while (true)
        {
            double max_score = iou_threshold_;
            int best_track = -1;
            int best_detection = -1;
            
            for (int i = 0; i < n_tracks; i++)
            {
                if (matched_tracks[i]) continue;
                for (int j = 0; j < n_detections; j++)
                {
                    if (matched_detections[j]) continue;
                    if (score_matrix[i][j] > max_score)
                    {
                        max_score = score_matrix[i][j];
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
        
        // 기존 트랙 업데이트
        for (int i = 0; i < n_tracks; i++)
        {
            if (track_to_detection[i] != -1)
            {
                // 매칭된 경우: 상태 업데이트
                int det_idx = track_to_detection[i];
                auto& det = detections.detections[det_idx];
                auto& track = tracks_[i];
                
                // 위치 히스토리 저장
                track.x_history.push_back(track.x);
                track.y_history.push_back(track.y);
                track.timestamps.push_back(timestamp);
                
                if (track.x_history.size() > history_size_)
                {
                    track.x_history.pop_front();
                    track.y_history.pop_front();
                    track.timestamps.pop_front();
                }
                
                // 위치 업데이트
                track.x = det.bbox.center.position.x;
                track.y = det.bbox.center.position.y;
                track.z = det.bbox.center.position.z;
                track.length = det.bbox.size.x;
                track.width = det.bbox.size.y;
                track.height = det.bbox.size.z;
                
                // 속도 계산
                if (track.x_history.size() >= 2)
                {
                    double dt = track.timestamps.back() - track.timestamps.front();
                    if (dt > 0.01)
                    {
                        track.vx = (track.x - track.x_history.front()) / dt;
                        track.vy = (track.y - track.y_history.front()) / dt;
                    }
                }
                
                track.age++;
                track.lost_frames = 0;
                
                // Detection에 track ID 저장
                det.results[0].id = track.id;
            }
            else
            {
                // 매칭 안 된 경우: 예측 위치로 업데이트
                auto& track = tracks_[i];
                
                if (track.lost_frames == 0 && track.x_history.size() >= 2)
                {
                    // 속도 기반 예측
                    double dt = 0.1;  // 10Hz 가정
                    track.x += track.vx * dt;
                    track.y += track.vy * dt;
                }
                
                track.lost_frames++;
            }
        }
        
        // 새로운 트랙 생성 (크기 필터 적용!)
        for (int j = 0; j < n_detections; j++)
        {
            if (detection_to_track[j] == -1)
            {
                auto& det = detections.detections[j];
                
                // 크기 체크: 너무 작은 클러스터는 노이즈일 가능성
                double size = det.bbox.size.x * det.bbox.size.y;
                if (size < min_size_for_track_)
                {
                    ROS_DEBUG("Skip small detection: size=%.3f m²", size);
                    continue;
                }
                
                TrackedObject new_track;
                new_track.id = next_id_++;
                new_track.x = det.bbox.center.position.x;
                new_track.y = det.bbox.center.position.y;
                new_track.z = det.bbox.center.position.z;
                new_track.length = det.bbox.size.x;
                new_track.width = det.bbox.size.y;
                new_track.height = det.bbox.size.z;
                new_track.age = 1;
                new_track.lost_frames = 0;
                
                tracks_.push_back(new_track);
                
                det.results[0].id = new_track.id;
            }
        }
        
        // Lost 트랙도 Detection으로 발행 (다중 필터 적용!)
        for (const auto& track : tracks_)
        {
            if (track.lost_frames > 0 && track.lost_frames <= max_lost_frames_)
            {
                double score = 1.0 - (track.lost_frames / (double)max_lost_frames_);
                
                // 나이 체크: 최소 3프레임 이상 살아있어야 함
                // 신뢰도 체크: 60% 이상만 발행
                if (track.age >= min_age_for_lost_track_ &&
                    score >= min_score_for_lost_track_)
                {
                    vision_msgs::Detection3D lost_det;
                    lost_det.bbox.center.position.x = track.x;
                    lost_det.bbox.center.position.y = track.y;
                    lost_det.bbox.center.position.z = track.z;
                    lost_det.bbox.center.orientation.w = 1.0;
                    lost_det.bbox.size.x = track.length;
                    lost_det.bbox.size.y = track.width;
                    lost_det.bbox.size.z = track.height;
                    
                    vision_msgs::ObjectHypothesisWithPose result;
                    result.id = track.id;
                    result.score = score;
                    lost_det.results.push_back(result);
                    
                    detections.detections.push_back(lost_det);
                }
            }
        }
        
        // 오래된 트랙 제거
        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                          [this](const TrackedObject& t) { 
                              return t.lost_frames > max_lost_frames_; 
                          }),
            tracks_.end()
        );
        
        int visible_tracks = std::count_if(tracks_.begin(), tracks_.end(), 
                                          [](const TrackedObject& t) { return t.lost_frames == 0; });
        int lost_tracks = std::count_if(tracks_.begin(), tracks_.end(), 
                                       [](const TrackedObject& t) { return t.lost_frames > 0; });
        
        ROS_INFO("Tracking: %lu total tracks (%d visible, %d lost)", 
                 tracks_.size(), visible_tracks, lost_tracks);
    }
    
    const std::vector<TrackedObject>& getTracks() const { return tracks_; }
};

#endif // SIMPLE_TRACKER_H