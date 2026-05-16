#include "SoftFilter.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>

// ---------------------------------------------------------------------------
// Configuration per resolution
// neighborhood = min similar 8-connected neighbors to survive
// threshold = max |depth_diff| between adjacent pixels to count as "similar"
// ---------------------------------------------------------------------------

SoftFilter::Config SoftFilter::getConfig(int width)
{
    Config cfg;
    if (width == 640) {           // VGA
        cfg.neighborhood = 2;     // need >= 2 similar neighbors to survive
        cfg.threshold = 200;      // max depth difference for similarity
    } else if (width == 1280) {   // SXGA
        cfg.neighborhood = 2;
        cfg.threshold = 4000;
    } else {                      // other
        cfg.neighborhood = 2;
        cfg.threshold = 90;
    }
    return cfg;
}

// ---------------------------------------------------------------------------
// FastSpeckleFilter — O(N) single-pass spatial coherence filter
//
// For each pixel with valid depth, count how many of its 8-connected neighbors
// have similar depth (within threshold). If the count is below neighborhood,
// the pixel is considered isolated noise and zeroed.
//
// Two-pass approach:
//   Pass 1: Mark pixels with insufficient similar neighbors
//   Pass 2: Zero marked pixels
//
// This replaces the BFS-based SoftFilter which was O(N) but with a much
// larger constant (40ms/frame on VGA). This implementation is ~2ms/frame.
//
// Principle: real depth edges have spatial coherence — a pixel at 3000mm
// surrounded by pixels at 500-900mm has no similar neighbors and is noise.
// A pixel at 800mm surrounded by pixels at 750-850mm has many similar
// neighbors and is valid, regardless of absolute depth.
// ---------------------------------------------------------------------------

void SoftFilter::apply(uint16_t* depth, int width, int height,
                       uint16_t noDepthValue)
{
    int totalPixels = width * height;
    Config cfg = getConfig(width);

    // Persistent thread-local mark buffer
    thread_local std::vector<uint8_t> mark;
    if (static_cast<int>(mark.size()) != totalPixels) {
        mark.resize(totalPixels, 0);
    }

    // Pass 1: Mark noise pixels
    memset(mark.data(), 0, totalPixels);

    const int minSimilar = cfg.neighborhood;
    const int thresh = cfg.threshold;

    for (int row = 1; row < height - 1; ++row) {
        const uint16_t* rowAbove = depth + (row - 1) * width;
        const uint16_t* rowCur   = depth + row * width;
        const uint16_t* rowBelow = depth + (row + 1) * width;
        uint8_t* markRow = mark.data() + row * width;

        for (int col = 1; col < width - 1; ++col) {
            uint16_t pixelDepth = rowCur[col];
            if (pixelDepth == noDepthValue) continue;

            int similarCount = 0;
            int pDepth = static_cast<int>(pixelDepth);

            // Check 8 neighbors (unrolled for speed)
            // Row above
            {
                int nDepth = static_cast<int>(rowAbove[col - 1]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            {
                int nDepth = static_cast<int>(rowAbove[col]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            {
                int nDepth = static_cast<int>(rowAbove[col + 1]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            // Same row
            {
                int nDepth = static_cast<int>(rowCur[col - 1]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            {
                int nDepth = static_cast<int>(rowCur[col + 1]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            // Row below
            {
                int nDepth = static_cast<int>(rowBelow[col - 1]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            {
                int nDepth = static_cast<int>(rowBelow[col]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }
            {
                int nDepth = static_cast<int>(rowBelow[col + 1]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth;
                    if (diff < 0) diff = -diff;
                    if (diff <= thresh) similarCount++;
                }
            }

            if (similarCount < minSimilar) {
                markRow[col] = 1;
            }
        }
    }

    // Handle edge pixels (row 0, row h-1, col 0, col w-1) — mark as noise
    // if they don't have enough neighbors. For simplicity, just mark edge
    // pixels with fewer than 1 similar neighbor (they have 3-5 neighbors).
    {
        // Row 0
        uint8_t* markRow = mark.data();
        const uint16_t* rowCur = depth;
        const uint16_t* rowBelow = depth + width;
        for (int col = 1; col < width - 1; ++col) {
            uint16_t p = rowCur[col];
            if (p == noDepthValue) continue;
            int pDepth = static_cast<int>(p);
            int count = 0;
            // 5 neighbors for top edge
            for (int dc = -1; dc <= 1; dc++) {
                int nDepth = static_cast<int>(rowCur[col + dc]);
                if (nDepth != noDepthValue && nDepth != pDepth) {
                    int diff = pDepth - nDepth; if (diff < 0) diff = -diff;
                    if (diff <= thresh) count++;
                }
                nDepth = static_cast<int>(rowBelow[col + dc]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth; if (diff < 0) diff = -diff;
                    if (diff <= thresh) count++;
                }
            }
            if (count < minSimilar) markRow[col] = 1;
        }
    }
    {
        // Last row
        uint8_t* markRow = mark.data() + (height - 1) * width;
        const uint16_t* rowAbove = depth + (height - 2) * width;
        const uint16_t* rowCur = depth + (height - 1) * width;
        for (int col = 1; col < width - 1; ++col) {
            uint16_t p = rowCur[col];
            if (p == noDepthValue) continue;
            int pDepth = static_cast<int>(p);
            int count = 0;
            for (int dc = -1; dc <= 1; dc++) {
                int nDepth = static_cast<int>(rowCur[col + dc]);
                if (nDepth != noDepthValue && nDepth != pDepth) {
                    int diff = pDepth - nDepth; if (diff < 0) diff = -diff;
                    if (diff <= thresh) count++;
                }
                nDepth = static_cast<int>(rowAbove[col + dc]);
                if (nDepth != noDepthValue) {
                    int diff = pDepth - nDepth; if (diff < 0) diff = -diff;
                    if (diff <= thresh) count++;
                }
            }
            if (count < minSimilar) markRow[col] = 1;
        }
    }

    // Pass 2: Apply marks
    int removed = 0;
    for (int i = 0; i < totalPixels; i++) {
        if (mark[i]) {
            depth[i] = noDepthValue;
            removed++;
        }
    }

    // Diagnostic (first 3 frames only, controlled by caller)
    static int diagCount = 0;
    if (diagCount < 3) {
        fprintf(stderr, "FastFilter: removed %d/%d pixels (%.1f%%)\n",
                removed, totalPixels, 100.0 * removed / totalPixels);
        diagCount++;
    }
}
