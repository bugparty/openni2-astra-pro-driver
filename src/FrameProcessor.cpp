#include "FrameProcessor.h"
#include <cstring>
#include <cstdio>

FrameProcessor::FrameProcessor(uint16_t sofType, uint16_t eofType)
    : sofType_(sofType)
    , eofType_(eofType)
    , lastSofPacketId_(0)
{
}

FrameProcessor::~FrameProcessor()
{
}

void FrameProcessor::setFrameReadyCallback(FrameReadyCallback cb)
{
    frameReadyCb_ = std::move(cb);
}

void FrameProcessor::processPacket(const PacketHeader& header,
                                    const uint8_t* data,
                                    uint32_t dataOffset,
                                    uint32_t dataSize)
{
    // Per-frame packet accounting
    framePacketCount_++;
    frameDataBytes_ += dataSize;

    // 1. SOF handling
    if (header.type == sofType_ && dataOffset == 0) {
        // Double-SOF detection: if packetID is sequential, it's a repeat SOF
        if (header.packetID == lastSofPacketId_ + 1) {
            return;  // Ignore duplicate SOF
        }

        // New frame starting. If we have data from a previous incomplete frame,
        // deliver it first (firmware may not send explicit EOF packets).
        if (bytesWritten_ > 0 && !frameCorrupted_) {
            onEndOfFrame(header);
        }

        lastSofPacketId_ = header.packetID;
        onStartOfFrame(header);
    }

    // 2. If frame not corrupted, let subclass process the data
    if (!frameCorrupted_) {
        processFramePacketChunk(header, data, dataOffset, dataSize);
    }

    // 3. Explicit EOF: end frame
    if (header.type == eofType_ && (dataOffset + dataSize) >= header.bufSize - sizeof(PacketHeader)) {
        onEndOfFrame(header);
    }
}

bool FrameProcessor::getLatestFrame(uint8_t** data, int* size)
{
    if (!newFrameReady_.exchange(false)) {
        // No new frame since last call
        return false;
    }

    std::lock_guard<std::mutex> lock(bufferMutex_);

    // Swap: stable gets what was ready, ready gets what was write
    int oldStable = stableIdx_;
    stableIdx_ = readyIdx_;
    readyIdx_ = writeIdx_;
    writeIdx_ = oldStable;

    // Return the stable buffer (the one we just made stable)
    *data = buffers_[stableIdx_].data.data();
    *size = buffers_[stableIdx_].size;
    return buffers_[stableIdx_].size > 0;
}

void FrameProcessor::setResolution(int width, int height)
{
    width_ = width;
    height_ = height;
}

void FrameProcessor::onStartOfFrame(const PacketHeader& /*header*/)
{
    frameCorrupted_ = false;
    bytesWritten_ = 0;
    framePacketCount_ = 0;
    frameDataBytes_ = 0;
}

void FrameProcessor::onEndOfFrame(const PacketHeader& header)
{
    bool wasCorrupted = frameCorrupted_;

    if (!wasCorrupted) {
        // Copy write buffer into the write slot of triple buffer
        std::lock_guard<std::mutex> lock(bufferMutex_);

        FrameBuffer& wb = buffers_[writeIdx_];
        if (wb.data.size() < static_cast<size_t>(bytesWritten_)) {
            wb.data.resize(bytesWritten_);
        }
        if (bytesWritten_ > 0) {
            memcpy(wb.data.data(), writeBuffer_.data(), bytesWritten_);
        }
        wb.size = bytesWritten_;
        wb.timestamp = header.timestamp;
        wb.frameId = ++frameCounter_;

        // Rotate: write becomes ready, ready becomes stable, stable becomes write
        int oldWrite = writeIdx_;
        int oldReady = readyIdx_;
        int oldStable = stableIdx_;

        readyIdx_ = oldWrite;
        stableIdx_ = oldReady;
        writeIdx_ = oldStable;

        // Signal that a new frame is ready
        newFrameReady_.store(true);
        if (frameReadyCb_) {
            frameReadyCb_();
        }
    }

    // Reset write state for next frame (corrupted or not)
    bytesWritten_ = 0;
    frameCorrupted_ = false;
}

void FrameProcessor::markCorrupted()
{
    if (!frameCorrupted_) {
        frameCorrupted_ = true;
    }
}
