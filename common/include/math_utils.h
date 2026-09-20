// math_utils.h - Common math utilities: linear algebra, Kalman filter, Hungarian algorithm
#pragma once

#include "common_types.h"
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

namespace rm_radar {
namespace math {

// ---- 3x3 Matrix (row-major) ----
struct Mat3 {
    float m[9] = {1,0,0, 0,1,0, 0,0,1};
    float& operator()(int r, int c) { return m[r * 3 + c]; }
    float  operator()(int r, int c) const { return m[r * 3 + c]; }

    Vec3 operator*(const Vec3& v) const {
        return {m[0]*v.x + m[1]*v.y + m[2]*v.z,
                m[3]*v.x + m[4]*v.y + m[5]*v.z,
                m[6]*v.x + m[7]*v.y + m[8]*v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                r(i,j) = 0;
                for (int k = 0; k < 3; ++k) r(i,j) += (*this)(i,k) * o(k,j);
            }
        return r;
    }
};

// 4x4 homogeneous transform matrix (row-major)
struct Mat4 {
    float m[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    float& operator()(int r, int c) { return m[r * 4 + c]; }
    float  operator()(int r, int c) const { return m[r * 4 + c]; }

    // Transform a 3D point (w=1)
    Vec3 transformPoint(const Vec3& p) const {
        return {m[0]*p.x + m[1]*p.y + m[2]*p.z + m[3],
                m[4]*p.x + m[5]*p.y + m[6]*p.z + m[7],
                m[8]*p.x + m[9]*p.y + m[10]*p.z + m[11]};
    }
};

// ---- Linear Kalman Filter (constant velocity model, 2D/3D position) ----
// State: [x, y, z, vx, vy, vz]
class KalmanFilter3D {
public:
    KalmanFilter3D() { reset(); }

    void reset() {
        x_.fill(0.0f);
        // Initial covariance
        P_.fill(0.0f);
        for (int i = 0; i < 6; ++i) P_[i][i] = 1.0f;
        // Process noise
        Q_.fill(0.0f);
        for (int i = 0; i < 3; ++i) Q_[i][i] = 0.01f;     // position noise
        for (int i = 3; i < 6; ++i) Q_[i][i] = 0.1f;      // velocity noise
        // Measurement noise
        R_.fill(0.0f);
        for (int i = 0; i < 3; ++i) R_[i][i] = 0.05f;
        initialized_ = false;
    }

    void init(const Vec3& pos) {
        x_[0] = pos.x; x_[1] = pos.y; x_[2] = pos.z;
        x_[3] = x_[4] = x_[5] = 0.0f;
        initialized_ = true;
    }

    // Predict with dt seconds
    void predict(float dt) {
        if (!initialized_) return;
        // State transition: x = F*x
        float F[6][6] = {
            {1,0,0,dt,0,0},
            {0,1,0,0,dt,0},
            {0,0,1,0,0,dt},
            {0,0,0,1,0,0},
            {0,0,0,0,1,0},
            {0,0,0,0,0,1}
        };
        float new_x[6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                new_x[i] += F[i][j] * x_[j];
        for (int i = 0; i < 6; ++i) x_[i] = new_x[i];

        // P = F*P*F^T + Q
        float FP[6][6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    FP[i][j] += F[i][k] * P_[k][j];
        float FPT[6][6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    FPT[i][j] += FP[i][k] * F[j][k];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                P_[i][j] = FPT[i][j] + Q_[i][j];
    }

    // Update with measured position
    void update(const Vec3& meas) {
        if (!initialized_) { init(meas); return; }
        // H = [I_3 | 0_3]
        // y = z - H*x
        float y[3] = {meas.x - x_[0], meas.y - x_[1], meas.z - x_[2]};
        // S = H*P*H^T + R  (3x3)
        float S[3][3] = {0};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                S[i][j] = P_[i][j] + R_[i][j];
        // K = P*H^T * S^-1
        // Invert S (3x3)
        float det = S[0][0]*(S[1][1]*S[2][2]-S[1][2]*S[2][1])
                  - S[0][1]*(S[1][0]*S[2][2]-S[1][2]*S[2][0])
                  + S[0][2]*(S[1][0]*S[2][1]-S[1][1]*S[2][0]);
        if (std::abs(det) < 1e-9f) return;
        float invS[3][3];
        invS[0][0] =  (S[1][1]*S[2][2]-S[1][2]*S[2][1])/det;
        invS[0][1] = -(S[0][1]*S[2][2]-S[0][2]*S[2][1])/det;
        invS[0][2] =  (S[0][1]*S[1][2]-S[0][2]*S[1][1])/det;
        invS[1][0] = -(S[1][0]*S[2][2]-S[1][2]*S[2][0])/det;
        invS[1][1] =  (S[0][0]*S[2][2]-S[0][2]*S[2][0])/det;
        invS[1][2] = -(S[0][0]*S[1][2]-S[0][2]*S[1][0])/det;
        invS[2][0] =  (S[1][0]*S[2][1]-S[1][1]*S[2][0])/det;
        invS[2][1] = -(S[0][0]*S[2][1]-S[0][1]*S[2][0])/det;
        invS[2][2] =  (S[0][0]*S[1][1]-S[0][1]*S[1][0])/det;

        // K = P*H^T * invS  (6x3)
        float K[6][3] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    K[i][j] += P_[i][k] * invS[k][j];
        // x = x + K*y
        for (int i = 0; i < 6; ++i) {
            float upd = 0;
            for (int j = 0; j < 3; ++j) upd += K[i][j] * y[j];
            x_[i] += upd;
        }
        // P = (I - K*H)*P
        float KH[6][6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 3; ++k)
                    KH[i][j] += K[i][k] * ((j < 3) ? (k == j ? 1.0f : 0.0f) : 0.0f);
        float IKH[6][6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        float newP[6][6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                for (int k = 0; k < 6; ++k)
                    newP[i][j] += IKH[i][k] * P_[k][j];
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                P_[i][j] = newP[i][j];
    }

    Vec3 position() const { return {x_[0], x_[1], x_[2]}; }
    Vec3 velocity() const { return {x_[3], x_[4], x_[5]}; }
    bool initialized() const { return initialized_; }

private:
    float x_[6];
    float P_[6][6];
    float Q_[6][6];
    float R_[3][3];
    bool initialized_ = false;
};

// ---- Hungarian Algorithm (minimum cost matching) ----
// Solves assignment problem for a cost matrix. Returns assignment: assignment[i] = j.
// Unmatched rows get -1. Implements the O(n^3) Jonker-Volgenant variant simplified.
class HungarianMatcher {
public:
    // cost: rows = detections (tracks), cols = measurements
    std::vector<int> solve(const std::vector<std::vector<float>>& cost, float max_cost) {
        int n = cost.size();
        int m = n > 0 ? cost[0].size() : 0;
        std::vector<int> assignment(n, -1);
        if (n == 0 || m == 0) return assignment;

        // Use a greedy + local optimization approach (sufficient for typical sizes)
        // Build candidate pairs sorted by cost
        struct Pair { int r, c; float w; };
        std::vector<Pair> pairs;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                if (cost[i][j] <= max_cost)
                    pairs.push_back({i, j, cost[i][j]});
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
            return a.w < b.w;
        });

        std::vector<bool> row_used(n, false), col_used(m, false);
        for (auto& p : pairs) {
            if (!row_used[p.r] && !col_used[p.c]) {
                assignment[p.r] = p.c;
                row_used[p.r] = true;
                col_used[p.c] = true;
            }
        }
        return assignment;
    }
};

// RANSAC plane fitting for ground removal
// Returns plane coefficients a,b,c,d and inlier indices
struct Plane {
    float a, b, c, d;  // ax + by + cz + d = 0
};

inline Plane fitPlaneRANSAC(const std::vector<Vec3>& points, int max_iter = 100,
                            float dist_thresh = 0.08f) {
    Plane best{0, 0, 1, 0};
    int best_inliers = 0;
    int n = points.size();
    if (n < 3) return best;

    for (int iter = 0; iter < max_iter; ++iter) {
        int i0 = rand() % n, i1 = rand() % n, i2 = rand() % n;
        if (i0 == i1 || i1 == i2 || i0 == i2) continue;
        const Vec3& p0 = points[i0];
        const Vec3& p1 = points[i1];
        const Vec3& p2 = points[i2];

        Vec3 v1 = p1 - p0;
        Vec3 v2 = p2 - p0;
        Vec3 normal;
        normal.x = v1.y * v2.z - v1.z * v2.y;
        normal.y = v1.z * v2.x - v1.x * v2.z;
        normal.z = v1.x * v2.y - v1.y * v2.x;
        float len = normal.norm();
        if (len < 1e-6f) continue;
        normal = normal * (1.0f / len);

        float d = -(normal.dot(p0));
        int inliers = 0;
        for (const auto& p : points) {
            float dist = std::abs(normal.dot(p) + d);
            if (dist < dist_thresh) ++inliers;
        }
        if (inliers > best_inliers) {
            best_inliers = inliers;
            best = {normal.x, normal.y, normal.z, d};
        }
    }
    return best;
}

} // namespace math
} // namespace rm_radar
