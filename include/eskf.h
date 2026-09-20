// eskf.h - Error State Kalman Filter for 3D drone trajectory estimation
// Nominal state: [x, y, z, vx, vy, vz] (position + velocity)
// Error state is estimated and injected back into the nominal state.
#pragma once

#include "common_types.h"
#include <array>
#include <cmath>

namespace rm_radar {
namespace anti_drone {

// Error State Kalman Filter
// Nominal state propagated by process model; error state corrected by measurements.
class ESKF {
public:
    ESKF() { reset(); }

    void reset() {
        for (auto& v : x_) v = 0.0f;
        for (auto& row : P_) for (auto& v : row) v = 0.0f;
        for (int i = 0; i < 6; ++i) P_[i][i] = 1.0f;
        // Process noise
        for (auto& row : Q_) for (auto& v : row) v = 0.0f;
        for (int i = 0; i < 3; ++i) Q_[i][i] = 0.005f;   // position error
        for (int i = 3; i < 6; ++i) Q_[i][i] = 0.05f;    // velocity error
        // Measurement noise
        for (auto& row : R_) for (auto& v : row) v = 0.0f;
        for (int i = 0; i < 3; ++i) R_[i][i] = 0.02f;
        initialized_ = false;
    }

    void init(const Vec3& pos, const Vec3& vel = Vec3(0,0,0)) {
        x_[0] = pos.x; x_[1] = pos.y; x_[2] = pos.z;
        x_[3] = vel.x; x_[4] = vel.y; x_[5] = vel.z;
        initialized_ = true;
    }

    // Predict nominal state with constant velocity model
    void predict(float dt) {
        if (!initialized_) return;
        // Nominal propagation: x += v*dt
        x_[0] += x_[3] * dt;
        x_[1] += x_[4] * dt;
        x_[2] += x_[5] * dt;

        // Error state covariance propagation: P = F*P*F^T + Q
        // F = [[I, dt*I], [0, I]] for 6-state
        float F[6][6] = {
            {1,0,0,dt,0,0},
            {0,1,0,0,dt,0},
            {0,0,1,0,0,dt},
            {0,0,0,1,0,0},
            {0,0,0,0,1,0},
            {0,0,0,0,0,1}
        };
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

    // Update with position measurement; compute error state and inject
    void update(const Vec3& meas) {
        if (!initialized_) { init(meas); return; }

        // Innovation (error): y = z - H*x (H selects position)
        float y[3] = {meas.x - x_[0], meas.y - x_[1], meas.z - x_[2]};

        // S = H*P*H^T + R  (3x3, H = [I_3 | 0])
        float S[3][3] = {0};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                S[i][j] = P_[i][j] + R_[i][j];

        // Invert S
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

        // Error state: dx = K * y
        float dx[6] = {0};
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j)
                dx[i] += K[i][j] * y[j];

        // Inject error into nominal state
        for (int i = 0; i < 6; ++i) x_[i] += dx[i];

        // P = (I - K*H) * P
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

        // Reset error state covariance (ESKF reset)
        for (int i = 0; i < 6; ++i) P_[i][i] = std::max(P_[i][i], 1e-4f);
    }

    Vec3 position() const { return {x_[0], x_[1], x_[2]}; }
    Vec3 velocity() const { return {x_[3], x_[4], x_[5]}; }
    bool initialized() const { return initialized_; }

    // Predicted position dt seconds ahead (for next-frame prediction)
    Vec3 predictPosition(float dt) const {
        return {x_[0] + x_[3]*dt, x_[1] + x_[4]*dt, x_[2] + x_[5]*dt};
    }

private:
    std::array<float, 6> x_;  // nominal state [x,y,z,vx,vy,vz]
    std::array<std::array<float, 6>, 6> P_;
    std::array<std::array<float, 6>, 6> Q_;
    std::array<std::array<float, 3>, 3> R_;
    bool initialized_ = false;
};

} // namespace anti_drone
} // namespace rm_radar
