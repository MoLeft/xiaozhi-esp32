#include "../motion_detector.h"

#include <cassert>

int main() {
    MotionDetector motion(MotionDetector::ScreenUpAxis::Y, false);

    assert(!motion.Update(0, -8192, 0, 0, true).rotation_changed);
    assert(!motion.Update(0, 8192, 0, 100, true).rotation_changed);
    auto inverted = motion.Update(0, 8192, 0, 650, true);
    assert(inverted.rotation_changed && inverted.inverted && !inverted.shake);

    assert(!motion.Update(0, 0, 8192, 800, true).rotation_changed);
    assert(!motion.Update(0, -8192, 0, 900, true).rotation_changed);
    auto restored = motion.Update(0, -8192, 0, 1450, true);
    assert(restored.rotation_changed && !restored.inverted && !restored.shake);

    assert(!motion.Update(15000, 0, 0, 2000, true).shake);
    assert(!motion.Update(0, -8192, 0, 2080, true).shake);
    assert(motion.Update(-15000, 0, 0, 2160, true).shake);
    assert(!motion.Update(0, -8192, 0, 2240, true).shake);
    assert(!motion.Update(15000, 0, 0, 2320, true).shake);

    MotionDetector busy(MotionDetector::ScreenUpAxis::Y, false);
    busy.Update(15000, 0, 0, 0, false);
    busy.Update(0, -8192, 0, 80, false);
    assert(!busy.Update(-15000, 0, 0, 160, false).shake);

    MotionDetector x_axis(MotionDetector::ScreenUpAxis::X, true);
    x_axis.Update(8192, 0, 0, 0, true);
    x_axis.Update(-8192, 0, 0, 100, true);
    assert(x_axis.Update(-8192, 0, 0, 650, true).inverted);

    MotionDetector topple(MotionDetector::ScreenUpAxis::X, true);
    topple.Update(8231, 713, 119, 0, true);
    topple.Update(8231, 713, 119, 480, true);
    assert(!topple.Update(3166, 3523, 8333, 560, true).toppled);
    assert(topple.Update(1908, 1008, 8043, 960, true).toppled);
    assert(!topple.Update(123, 682, 8298, 1200, true).toppled);
    topple.Update(8235, 622, 116, 2000, true);
    topple.Update(8235, 622, 116, 2480, true);
    assert(!topple.Update(-1346, 1810, 6529, 2520, true).toppled);
    assert(!topple.Update(547, 105, -9635, 2560, true).toppled);
    assert(topple.Update(124, 649, -8111, 2880, true).toppled);

    MotionDetector busy_topple(MotionDetector::ScreenUpAxis::X, true);
    busy_topple.Update(-8192, 0, 0, 0, false);
    busy_topple.Update(-8192, 0, 0, 480, false);
    busy_topple.Update(0, 0, -8192, 560, false);
    assert(!busy_topple.Update(0, 0, -8192, 960, false).toppled);
    assert(!busy_topple.Update(0, 0, -8192, 1040, true).toppled);

    MotionDetector stale_topple(MotionDetector::ScreenUpAxis::X, true);
    stale_topple.Update(8192, 0, 0, 0, true);
    stale_topple.Update(8192, 0, 0, 480, true);
    stale_topple.Update(0, 8192, 0, 3000, true);
    stale_topple.Update(0, 0, 8192, 3080, true);
    assert(!stale_topple.Update(0, 0, 8192, 3480, true).toppled);
}
