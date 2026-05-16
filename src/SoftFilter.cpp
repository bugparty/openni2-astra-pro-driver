#include "SoftFilter.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <algorithm>

SoftFilter::Config SoftFilter::getConfig(int width)
{
    Config cfg;
    if (width == 640) {           // VGA
        cfg.maxDiff = 4;
        cfg.maxSpeckleSize = 240;
    } else if (width == 1280) {   // SXGA
        cfg.maxDiff = 5;
        cfg.maxSpeckleSize = 4000;
    } else {
        cfg.maxDiff = 5;
        cfg.maxSpeckleSize = 90;
    }
    return cfg;
}

// Flood-fill BFS speckle filter, matching official Softfilter @ 0x98849.
//
// Layout of working buffer (totalPixels * 7 bytes):
//   [0 .. totalPixels*4)       = int32 labels (one per pixel)
//   [totalPixels*4 .. tp*8)    = int32 stack (for flood-fill BFS)
//   [tp*4 + tp*4 .. tp*4+tp*4+tp*1) = uint8 flags (one per label)
//
// Simplified: use separate vectors for clarity since we're not
// memory-constrained like the original which packed everything into
// one allocation.
void SoftFilter::apply(uint16_t* depth, int width, int height,
                       uint16_t noDepthValue)
{
    int totalPixels = width * height;
    Config cfg = getConfig(width);

    // Labels: int32 per pixel, 0 = unlabeled
    thread_local std::vector<int32_t> labels;
    labels.assign(totalPixels, 0);

    // Flags: byte per label, 0 = valid region, 1 = speckle
    // Max possible labels = totalPixels, but typically << that.
    thread_local std::vector<uint8_t> flags;
    if (static_cast<int>(flags.size()) < totalPixels + 1) {
        flags.resize(totalPixels + 1, 0);
    }

    // BFS stack: stores packed (col | (row << 16))
    thread_local std::vector<uint32_t> stack;
    stack.resize(totalPixels);

    int labelCount = 0;

    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {
            int idx = row * width + col;

            if (depth[idx] == noDepthValue) continue;

            if (labels[idx] == 0) {
                // Start flood fill
                labelCount++;
                int regionSize = 0;
                labels[idx] = labelCount;

                int stackPtr = 0;  // stack top
                stack[stackPtr++] = static_cast<uint32_t>(col | (row << 16));

                while (stackPtr > 0) {
                    regionSize++;

                    uint32_t pos = stack[--stackPtr];
                    int curCol = static_cast<int16_t>(pos & 0xFFFF);
                    int curRow = static_cast<int16_t>((pos >> 16) & 0xFFFF);
                    int curIdx = curRow * width + curCol;
                    uint16_t curDepth = depth[curIdx];

                    // Check 4 neighbors
                    // Right
                    if (curCol < width - 1) {
                        int nIdx = curIdx + 1;
                        if (labels[nIdx] == 0 && depth[nIdx] != noDepthValue &&
                            std::abs(static_cast<int>(curDepth) - static_cast<int>(depth[nIdx])) <= cfg.maxDiff) {
                            labels[nIdx] = labelCount;
                            stack[stackPtr++] = static_cast<uint32_t>((curCol + 1) | (curRow << 16));
                        }
                    }
                    // Left
                    if (curCol > 0) {
                        int nIdx = curIdx - 1;
                        if (labels[nIdx] == 0 && depth[nIdx] != noDepthValue &&
                            std::abs(static_cast<int>(curDepth) - static_cast<int>(depth[nIdx])) <= cfg.maxDiff) {
                            labels[nIdx] = labelCount;
                            stack[stackPtr++] = static_cast<uint32_t>((curCol - 1) | (curRow << 16));
                        }
                    }
                    // Down
                    if (curRow < height - 1) {
                        int nIdx = curIdx + width;
                        if (labels[nIdx] == 0 && depth[nIdx] != noDepthValue &&
                            std::abs(static_cast<int>(curDepth) - static_cast<int>(depth[nIdx])) <= cfg.maxDiff) {
                            labels[nIdx] = labelCount;
                            stack[stackPtr++] = static_cast<uint32_t>(curCol | ((curRow + 1) << 16));
                        }
                    }
                    // Up
                    if (curRow > 0) {
                        int nIdx = curIdx - width;
                        if (labels[nIdx] == 0 && depth[nIdx] != noDepthValue &&
                            std::abs(static_cast<int>(curDepth) - static_cast<int>(depth[nIdx])) <= cfg.maxDiff) {
                            labels[nIdx] = labelCount;
                            stack[stackPtr++] = static_cast<uint32_t>(curCol | ((curRow - 1) << 16));
                        }
                    }
                }

                // Classify region
                if (regionSize <= cfg.maxSpeckleSize) {
                    // Speckle
                    if (labelCount < static_cast<int>(flags.size())) {
                        flags[labelCount] = 1;
                    }
                    depth[idx] = noDepthValue;
                } else {
                    // Valid
                    if (labelCount < static_cast<int>(flags.size())) {
                        flags[labelCount] = 0;
                    }
                }
            } else {
                // Already labeled — check flag
                int lbl = labels[idx];
                if (lbl > 0 && lbl < static_cast<int>(flags.size()) && flags[lbl] != 0) {
                    depth[idx] = noDepthValue;
                }
            }
        }
    }
}
