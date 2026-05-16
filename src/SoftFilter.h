#pragma once

#include <cstdint>
#include <vector>

// SoftFilter: BFS-based depth speckle removal algorithm.
// Reverse-engineered from Orbbec official driver (liborbbec.so _Z10SoftfilterPhPtii).
//
// Removes small isolated clusters of valid depth pixels (speckle noise).
// A cluster is considered "small" if the BFS from any of its pixels visits
// fewer than `neighborhood` other pixels before exhausting all 4-connected
// neighbors with similar depth (|diff| <= threshold).
//
// Parameters by resolution (from disassembly):
//   VGA (640):  neighborhood=4, threshold=240
//   SXGA (1280): neighborhood=5, threshold=4000
//   other:       neighborhood=5, threshold=90
//
// NOTE: This is a speckle REMOVAL filter, not a hole-filling filter.
// It will DECREASE the number of valid pixels by removing noise.

class SoftFilter {
public:
    // Configuration parameters per resolution
    struct Config {
        int neighborhood;  // max BFS expansion steps for "small" cluster
        int threshold;     // max |depth_diff| between adjacent pixels
    };

    // Get configuration for a given frame width
    static Config getConfig(int width);

    // Apply SoftFilter in-place on a depth buffer.
    // depth: uint16_t array of width*height pixels (0 = no depth)
    // width, height: frame dimensions
    // noDepthValue: sentinel value for invalid pixels (default 0)
    static void apply(uint16_t* depth, int width, int height,
                      uint16_t noDepthValue = 0);
};
