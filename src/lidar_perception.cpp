// lidar_perception.cpp - Implementation of the LiDAR perception pipeline
#include "lidar_perception.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>
#include <unordered_set>

namespace rm_radar {
namespace lidar {

using PCLPoint = pcl::PointXYZI;

// Convert internal PointCloud to PCL point cloud
static pcl::PointCloud<PCLPoint>::Ptr toPCL(const PointCloud& in) {
    auto out = pcl::make_shared<pcl::PointCloud<PCLPoint>>();
    out->reserve(in.points.size());
    for (const auto& p : in.points) {
        PCLPoint pt;
        pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.intensity = p.intensity;
        out->push_back(pt);
    }
    out->width = out->size();
    out->height = 1;
    out->is_dense = false;
    return out;
}

// Convert PCL cloud back to internal PointCloud
static PointCloud fromPCL(const pcl::PointCloud<PCLPoint>& in) {
    PointCloud out;
    out.points.reserve(in.size());
    for (const auto& p : in.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        out.points.emplace_back(p.x, p.y, p.z, p.intensity, 0);
    }
    return out;
}

LidarPerception::LidarPerception() = default;
LidarPerception::~LidarPerception() = default;

bool LidarPerception::loadConfig(const std::string& yaml_path) {
    if (!config_loader_.load(yaml_path)) {
        RM_LOG_WARN("Failed to load config: " + yaml_path);
        return false;
    }
    cfg_.x_min = config_loader_.getFloat("x_min", cfg_.x_min);
    cfg_.x_max = config_loader_.getFloat("x_max", cfg_.x_max);
    cfg_.y_min = config_loader_.getFloat("y_min", cfg_.y_min);
    cfg_.y_max = config_loader_.getFloat("y_max", cfg_.y_max);
    cfg_.z_min = config_loader_.getFloat("z_min", cfg_.z_min);
    cfg_.z_max = config_loader_.getFloat("z_max", cfg_.z_max);
    cfg_.voxel_leaf = config_loader_.getFloat("leaf_size", cfg_.voxel_leaf);
    cfg_.voxel_leaf_far = config_loader_.getFloat("leaf_size_far", cfg_.voxel_leaf_far);
    cfg_.far_dist_threshold = config_loader_.getFloat("far_distance_threshold", cfg_.far_dist_threshold);
    cfg_.ground_dist_thresh = config_loader_.getFloat("distance_threshold", cfg_.ground_dist_thresh);
    cfg_.ground_max_iter = config_loader_.getInt("max_iterations", cfg_.ground_max_iter);
    cfg_.ground_optimize_z_below = config_loader_.getFloat("optimize_z_below", cfg_.ground_optimize_z_below);
    cfg_.cluster_base_radius = config_loader_.getFloat("base_radius", cfg_.cluster_base_radius);
    cfg_.cluster_radius_gain = config_loader_.getFloat("radius_gain", cfg_.cluster_radius_gain);
    cfg_.cluster_min_size = config_loader_.getInt("min_cluster_size", cfg_.cluster_min_size);
    cfg_.cluster_max_size = config_loader_.getInt("max_cluster_size", cfg_.cluster_max_size);
    cfg_.max_match_distance = config_loader_.getFloat("max_match_distance", cfg_.max_match_distance);
    cfg_.max_lost_frames = config_loader_.getInt("max_lost_frames", cfg_.max_lost_frames);
    cfg_.static_filter_enabled = config_loader_.getBool("enabled", cfg_.static_filter_enabled);
    cfg_.publish_udp = config_loader_.getBool("publish_udp", cfg_.publish_udp);
    cfg_.udp_host = config_loader_.get("udp_host", cfg_.udp_host);
    cfg_.udp_port = config_loader_.getInt("udp_port", cfg_.udp_port);
    cfg_.visualization_enabled = config_loader_.getBool("enabled", cfg_.visualization_enabled);
    RM_LOG_INFO("Lidar config loaded from " + yaml_path);
    return true;
}

bool LidarPerception::init() {
    initialized_ = true;
    RM_LOG_INFO("LidarPerception initialized");
    return true;
}

void LidarPerception::setStaticObstacles(const std::vector<StaticObstacle>& obs) {
    cfg_.static_obstacles = obs;
}

void LidarPerception::setExtrinsics(const std::vector<std::array<float, 16>>& ext) {
    cfg_.extrinsics = ext;
}

PointCloud LidarPerception::passthrough(const PointCloud& in) {
    PointCloud out;
    out.timestamp_us = in.timestamp_us;
    out.frame_id = in.frame_id;
    out.points.reserve(in.points.size());
    for (const auto& p : in.points) {
        if (p.x >= cfg_.x_min && p.x <= cfg_.x_max &&
            p.y >= cfg_.y_min && p.y <= cfg_.y_max &&
            p.z >= cfg_.z_min && p.z <= cfg_.z_max) {
            out.points.push_back(p);
        }
    }
    return out;
}

PointCloud LidarPerception::voxelDownsample(const PointCloud& in) {
    auto cloud = toPCL(in);
    if (cloud->empty()) return PointCloud();

    // Determine leaf size: relax for distant clouds
    float leaf = cfg_.voxel_leaf;
    if (!in.points.empty()) {
        float max_dist = 0.0f;
        for (const auto& p : in.points) {
            float d = std::sqrt(p.x * p.x + p.y * p.y);
            if (d > max_dist) max_dist = d;
        }
        if (max_dist > cfg_.far_dist_threshold) {
            leaf = cfg_.voxel_leaf_far;
        }
    }

    pcl::VoxelGrid<PCLPoint> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(leaf, leaf, leaf);
    auto filtered = pcl::make_shared<pcl::PointCloud<PCLPoint>>();
    vox.filter(*filtered);
    return fromPCL(*filtered);
}

PointCloud LidarPerception::removeGround(const PointCloud& in) {
    auto cloud = toPCL(in);
    if (cloud->empty()) return PointCloud();

    // Optimization: only run RANSAC on points with z < threshold (ground candidates)
    pcl::PointIndices ground_candidates;
    pcl::PointIndices non_ground_candidates;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if ((*cloud)[i].z < cfg_.ground_optimize_z_below) {
            ground_candidates.indices.push_back(static_cast<int>(i));
        } else {
            non_ground_candidates.indices.push_back(static_cast<int>(i));
        }
    }

    // Fit ground plane on the low-z candidates
    pcl::PointCloud<PCLPoint> ground_cloud;
    for (int idx : ground_candidates.indices) {
        ground_cloud.push_back((*cloud)[idx]);
    }

    pcl::SACSegmentation<PCLPoint> seg;
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeffs;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(cfg_.ground_max_iter);
    seg.setDistanceThreshold(cfg_.ground_dist_thresh);

    auto ground_ptr = pcl::make_shared<pcl::PointCloud<PCLPoint>>(ground_cloud);
    seg.setInputCloud(ground_ptr);
    seg.segment(inliers, coeffs);

    // Build set of ground inlier indices in the original cloud
    std::unordered_set<int> ground_set;
    for (int local_idx : inliers.indices) {
        if (local_idx < (int)ground_candidates.indices.size()) {
            ground_set.insert(ground_candidates.indices[local_idx]);
        }
    }

    // Also include any non-candidate point that lies on the fitted plane
    if (!coeffs.values.empty()) {
        float a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
        float norm = std::sqrt(a * a + b * b + c * c);
        if (norm > 1e-6f) {
            for (int idx : non_ground_candidates.indices) {
                const auto& p = (*cloud)[idx];
                float dist = std::abs(a * p.x + b * p.y + c * p.z + d) / norm;
                if (dist < cfg_.ground_dist_thresh) {
                    ground_set.insert(idx);
                }
            }
        }
    }

    PointCloud out;
    out.timestamp_us = in.timestamp_us;
    out.frame_id = in.frame_id;
    out.points.reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (ground_set.find((int)i) == ground_set.end()) {
            const auto& p = (*cloud)[i];
            out.points.emplace_back(p.x, p.y, p.z, p.intensity, 0);
        }
    }
    return out;
}

BoundingBox3D LidarPerception::computeBBox(const std::vector<PointXYZIR>& pts) const {
    BoundingBox3D bb;
    if (pts.empty()) return bb;
    float min_x = pts[0].x, max_x = pts[0].x;
    float min_y = pts[0].y, max_y = pts[0].y;
    float min_z = pts[0].z, max_z = pts[0].z;
    float cx = 0, cy = 0, cz = 0;
    for (const auto& p : pts) {
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        cx += p.x; cy += p.y; cz += p.z;
    }
    cx /= pts.size(); cy /= pts.size(); cz /= pts.size();
    bb.center = Vec3(cx, cy, cz);
    bb.size = Vec3(max_x - min_x, max_y - min_y, max_z - min_z);
    bb.yaw = 0.0f;
    return bb;
}

std::vector<Cluster> LidarPerception::cluster(const PointCloud& in) {
    std::vector<Cluster> result;
    auto cloud = toPCL(in);
    if (cloud->empty()) return result;

    pcl::search::KdTree<PCLPoint>::Ptr tree(new pcl::search::KdTree<PCLPoint>());
    tree->setInputCloud(cloud);

    // Compute per-point distance to determine dynamic clustering radius
    std::vector<float> dists(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = (*cloud)[i];
        dists[i] = std::sqrt(p.x * p.x + p.y * p.y);
    }

    // Use the mean distance as the representative distance for cluster radius
    float mean_dist = 0.0f;
    if (!dists.empty()) {
        for (float d : dists) mean_dist += d;
        mean_dist /= dists.size();
    }
    float cluster_radius = cfg_.cluster_base_radius + cfg_.cluster_radius_gain * mean_dist;

    pcl::EuclideanClusterExtraction<PCLPoint> ec;
    ec.setClusterTolerance(cluster_radius);
    ec.setMinClusterSize(cfg_.cluster_min_size);
    ec.setMaxClusterSize(cfg_.cluster_max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    for (const auto& ci : cluster_indices) {
        Cluster c;
        c.points.reserve(ci.indices.size());
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, ci.indices, centroid);
        c.centroid = Vec3(centroid[0], centroid[1], centroid[2]);
        c.distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);

        float intensity_sum = 0.0f;
        for (int idx : ci.indices) {
            const auto& p = (*cloud)[idx];
            c.points.emplace_back(p.x, p.y, p.z, p.intensity, 0);
            intensity_sum += p.intensity;
        }
        c.point_count = c.points.size();
        c.avg_intensity = c.points.empty() ? 0.0f : intensity_sum / c.points.size();
        c.bbox = computeBBox(c.points);

        // Static obstacle filter
        if (cfg_.static_filter_enabled && isStaticObstacle(c.centroid)) {
            continue;
        }
        result.push_back(std::move(c));
    }
    return result;
}

bool LidarPerception::isStaticObstacle(const Vec3& pos) const {
    for (const auto& obs : cfg_.static_obstacles) {
        float dx = pos.x - obs.position.x;
        float dy = pos.y - obs.position.y;
        if (std::sqrt(dx * dx + dy * dy) < obs.radius) {
            return true;
        }
    }
    return false;
}

void LidarPerception::track(const std::vector<Cluster>& clusters, std::vector<Target>& out_targets) {
    // Build cost matrix: rows = tracks, cols = clusters
    int n_tracks = static_cast<int>(tracks_.size());
    int n_meas = static_cast<int>(clusters.size());

    std::vector<std::vector<float>> cost(n_tracks, std::vector<float>(n_meas, 1e9f));
    for (int i = 0; i < n_tracks; ++i) {
        if (!tracks_[i].active) continue;
        for (int j = 0; j < n_meas; ++j) {
            float dx = tracks_[i].position.x - clusters[j].centroid.x;
            float dy = tracks_[i].position.y - clusters[j].centroid.y;
            float dz = tracks_[i].position.z - clusters[j].centroid.z;
            cost[i][j] = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    // Hungarian matching
    math::HungarianMatcher matcher;
    std::vector<int> assignment = matcher.solve(cost, cfg_.max_match_distance);

    std::vector<bool> meas_matched(n_meas, false);

    // Update matched tracks
    for (int i = 0; i < n_tracks; ++i) {
        if (assignment[i] >= 0) {
            int j = assignment[i];
            meas_matched[j] = true;
            tracks_[i].kf.predict(0.1f); // assume ~100ms frame interval
            tracks_[i].kf.update(clusters[j].centroid);
            tracks_[i].position = tracks_[i].kf.position();
            tracks_[i].velocity = tracks_[i].kf.velocity();
            tracks_[i].bbox = clusters[j].bbox;
            tracks_[i].point_count = clusters[j].point_count;
            tracks_[i].avg_intensity = clusters[j].avg_intensity;
            tracks_[i].lost_frames = 0;
            tracks_[i].confidence = std::min(1.0f, tracks_[i].confidence + 0.2f);
        } else {
            // Track not matched
            tracks_[i].lost_frames++;
            tracks_[i].kf.predict(0.1f);
            tracks_[i].position = tracks_[i].kf.position();
            tracks_[i].velocity = tracks_[i].kf.velocity();
            if (tracks_[i].lost_frames > cfg_.max_lost_frames) {
                tracks_[i].active = false;
            }
        }
    }

    // Create new tracks for unmatched measurements
    for (int j = 0; j < n_meas; ++j) {
        if (!meas_matched[j]) {
            TrackedTarget t;
            t.id = next_track_id_++;
            t.position = clusters[j].centroid;
            t.velocity = Vec3(0, 0, 0);
            t.bbox = clusters[j].bbox;
            t.point_count = clusters[j].point_count;
            t.avg_intensity = clusters[j].avg_intensity;
            t.confidence = 0.3f;
            t.kf.init(clusters[j].centroid);
            tracks_.push_back(t);
        }
    }

    // Remove inactive tracks
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
        [](const TrackedTarget& t) { return !t.active; }), tracks_.end());

    // Build output targets
    for (const auto& t : tracks_) {
        if (!t.active) continue;
        Target target;
        target.id = t.id;
        target.type = TargetType::UNKNOWN;
        target.position = t.position;
        target.velocity = t.velocity;
        target.confidence = t.confidence;
        target.timestamp_us = now_us();
        target.point_count = t.point_count;
        target.bbox[0] = t.bbox.center.x - t.bbox.size.x * 0.5f;
        target.bbox[1] = t.bbox.center.y - t.bbox.size.y * 0.5f;
        target.bbox[2] = t.bbox.center.z - t.bbox.size.z * 0.5f;
        target.bbox[3] = t.bbox.center.x + t.bbox.size.x * 0.5f;
        target.bbox[4] = t.bbox.center.y + t.bbox.size.y * 0.5f;
        target.bbox[5] = t.bbox.center.z + t.bbox.size.z * 0.5f;
        out_targets.push_back(target);
    }
}

PerceptionResult LidarPerception::process(const PointCloud& input) {
    auto t0 = std::chrono::steady_clock::now();

    PerceptionResult result;
    result.timestamp_us = input.timestamp_us;

    // Stage 1: Passthrough filter
    PointCloud pc = passthrough(input);

    // Stage 2: Voxel downsampling
    pc = voxelDownsample(pc);

    // Stage 3: RANSAC ground removal
    pc = removeGround(pc);
    result.filtered_cloud = pc;

    // Stage 4: Euclidean clustering
    auto clusters = cluster(pc);
    result.clusters = clusters;

    // Stage 5: Tracking (Hungarian + Kalman)
    track(clusters, result.targets);

    auto t1 = std::chrono::steady_clock::now();
    result.processing_time_ms =
        std::chrono::duration<float, std::milli>(t1 - t0).count();

    return result;
}

} // namespace lidar
} // namespace rm_radar
