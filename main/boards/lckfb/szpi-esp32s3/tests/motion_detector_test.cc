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
}
