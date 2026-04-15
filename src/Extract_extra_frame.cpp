#include "Extract_extra_frame.h"

#include <cmath>

SelectedBufferedFrames selectEvenCenteredFrames(
    const std::vector<BufferedFrame>& frames,
    int num_frames,
    bool preferLeftOnExactCenter)
{
    const int n = static_cast<int>(frames.size());

    if (num_frames <= 0 || n == 0) {
        return {};
    }

    if (num_frames >= n) {
        return {frames};
    }

    SelectedBufferedFrames result;
    result.frames.reserve(num_frames);

    const double globalCenter = (n - 1) / 2.0;

    for (int k = 0; k < num_frames; ++k) {
        const double target =
            (k + 0.5) * n / static_cast<double>(num_frames) - 0.5;

        int low = static_cast<int>(std::floor(target));
        int high = static_cast<int>(std::ceil(target));

        if (low < 0) low = 0;
        if (high >= n) high = n - 1;

        int idx;
        const double dLow = std::abs(target - low);
        const double dHigh = std::abs(target - high);

        if (dLow < dHigh) {
            idx = low;
        } else if (dHigh < dLow) {
            idx = high;
        } else {
            if (target < globalCenter) {
                idx = low;
            } else if (target > globalCenter) {
                idx = high;
            } else {
                idx = preferLeftOnExactCenter ? low : high;
            }
        }

        result.frames.push_back(frames[idx]);
    }

    return result;
}