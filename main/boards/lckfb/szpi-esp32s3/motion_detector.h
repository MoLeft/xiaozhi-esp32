#ifndef LICHUANG_MOTION_DETECTOR_H
#define LICHUANG_MOTION_DETECTOR_H

#include <cstdint>

struct MotionDecision {
    bool rotation_changed = false;
    bool inverted = false;
    bool shake = false;
};

class MotionDetector {
public:
    enum class ScreenUpAxis { X, Y };

    MotionDetector(ScreenUpAxis axis, bool normal_positive)
        : axis_(axis), normal_positive_(normal_positive) {}

    MotionDecision Update(int16_t ax, int16_t ay, int16_t az, uint32_t now_ms, bool allow_wake) {
        MotionDecision decision;
        const int32_t up = axis_ == ScreenUpAxis::X ? ax : ay;
        const int32_t side = axis_ == ScreenUpAxis::X ? ay : ax;
        const int64_t magnitude_squared = int64_t(ax) * ax + int64_t(ay) * ay + int64_t(az) * az;
        const int32_t abs_up = up < 0 ? -up : up;
        const int32_t abs_side = side < 0 ? -side : side;
        const int32_t abs_z = az < 0 ? -int32_t(az) : az;

        // Hold the last orientation while flat, moving, or near a side edge.
        if (magnitude_squared >= int64_t(5000) * 5000 &&
            magnitude_squared <= int64_t(11000) * 11000 && abs_up >= 5500 &&
            abs_up * 4 >= abs_side * 5 && abs_up * 4 >= abs_z * 5) {
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

        const bool impulse = magnitude_squared >= int64_t(13000) * 13000;
        if (!allow_wake) {
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
    bool was_impulse_ = false;
    uint8_t impulse_count_ = 0;
    uint32_t last_impulse_ms_ = 0;
    bool has_shaken_ = false;
    uint32_t last_shake_ms_ = 0;
};

#endif
