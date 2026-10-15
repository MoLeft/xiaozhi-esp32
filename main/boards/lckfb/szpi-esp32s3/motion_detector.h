#ifndef LICHUANG_MOTION_DETECTOR_H
#define LICHUANG_MOTION_DETECTOR_H

#include <cstdint>

enum class MotionPose { Unknown, Upright, Inverted, FaceUp, FaceDown, MovingOrTilted };

struct MotionDecision {
    bool rotation_changed = false;
    bool inverted = false;
    bool shake = false;
    bool toppled = false;
    MotionPose pose = MotionPose::Unknown;
};

class MotionDetector {
public:
    enum class ScreenUpAxis { X, Y };

    MotionDetector(ScreenUpAxis axis, bool normal_positive)
        : axis_(axis), normal_positive_(normal_positive) {}

    MotionDecision Update(int16_t ax, int16_t ay, int16_t az, uint32_t now_ms, bool allow_wake,
                          bool allow_shake = true, bool allow_topple = true) {
        MotionDecision decision;
        const int32_t up = axis_ == ScreenUpAxis::X ? ax : ay;
        const int32_t side = axis_ == ScreenUpAxis::X ? ay : ax;
        const int64_t magnitude_squared = int64_t(ax) * ax + int64_t(ay) * ay + int64_t(az) * az;
        const int32_t abs_up = up < 0 ? -up : up;
        const int32_t abs_side = side < 0 ? -side : side;
        const int32_t abs_z = az < 0 ? -int32_t(az) : az;

        const bool gravity = magnitude_squared >= int64_t(5000) * 5000 &&
                             magnitude_squared <= int64_t(11000) * 11000;
        const bool upright =
            gravity && abs_up >= 5500 && abs_up * 4 >= abs_side * 5 && abs_up * 4 >= abs_z * 5;
        const bool flat =
            gravity && abs_z >= 6000 && abs_z * 4 >= abs_up * 5 && abs_z * 4 >= abs_side * 5;
        decision.pose =
            upright ? ((up > 0) == normal_positive_ ? MotionPose::Upright : MotionPose::Inverted)
            : flat  ? (az > 0 ? MotionPose::FaceUp : MotionPose::FaceDown)
                    : MotionPose::MovingOrTilted;

        // Hold the last orientation while flat, moving, or near a side edge.
        if (upright) {
            const bool inverted = (up > 0) != normal_positive_;
            if (inverted == inverted_) {
                candidate_valid_ = false;
            } else if (!candidate_valid_ || candidate_inverted_ != inverted) {
                candidate_valid_ = true;
                candidate_inverted_ = inverted;
                candidate_since_ms_ = now_ms;
            } else if (now_ms - candidate_since_ms_ >= 500) {
                inverted_ = inverted;
                candidate_valid_ = false;
                decision.rotation_changed = true;
            }
        } else {
            candidate_valid_ = false;
        }
        decision.inverted = inverted_;

        if (!allow_topple) {
            upright_candidate_ = false;
            topple_armed_ = false;
            flat_candidate_ = false;
        } else if (upright) {
            if (!upright_candidate_) {
                upright_candidate_ = true;
                upright_since_ms_ = now_ms;
            } else if (now_ms - upright_since_ms_ >= 400) {
                topple_armed_ = true;
                last_upright_ms_ = now_ms;
            }
            flat_candidate_ = false;
        } else {
            upright_candidate_ = false;
            if (topple_armed_ && now_ms - last_upright_ms_ > 2000) {
                topple_armed_ = false;
            }
            if (topple_armed_ && flat) {
                if (!flat_candidate_ || flat_face_up_ != (az > 0)) {
                    flat_candidate_ = true;
                    flat_face_up_ = az > 0;
                    flat_since_ms_ = now_ms;
                } else if (now_ms - flat_since_ms_ >= 320) {
                    topple_armed_ = false;
                    flat_candidate_ = false;
                    decision.toppled = allow_wake;
                }
            } else {
                flat_candidate_ = false;
            }
        }

        const bool impulse = magnitude_squared >= int64_t(13000) * 13000;
        if (!allow_wake || !allow_shake) {
            impulse_count_ = 0;
        } else if (impulse && !was_impulse_) {
            if (impulse_count_ == 0 || now_ms - last_impulse_ms_ > 700) {
                impulse_count_ = 1;
            } else if (++impulse_count_ >= 2) {
                impulse_count_ = 0;
                if (!has_shaken_ || now_ms - last_shake_ms_ >= 3000) {
                    has_shaken_ = true;
                    last_shake_ms_ = now_ms;
                    decision.shake = true;
                }
            }
            last_impulse_ms_ = now_ms;
        }
        was_impulse_ = impulse;
        return decision;
    }

private:
    ScreenUpAxis axis_;
    bool normal_positive_;
    bool inverted_ = false;
    bool candidate_valid_ = false;
    bool candidate_inverted_ = false;
    uint32_t candidate_since_ms_ = 0;
    bool upright_candidate_ = false;
    uint32_t upright_since_ms_ = 0;
    bool topple_armed_ = false;
    uint32_t last_upright_ms_ = 0;
    bool flat_candidate_ = false;
    bool flat_face_up_ = false;
    uint32_t flat_since_ms_ = 0;
    bool was_impulse_ = false;
    uint8_t impulse_count_ = 0;
    uint32_t last_impulse_ms_ = 0;
    bool has_shaken_ = false;
    uint32_t last_shake_ms_ = 0;
};

#endif
