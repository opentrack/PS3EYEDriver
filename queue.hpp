#pragma once

#include "internal.hpp"
#include <climits>

#include <cstdint>
#include <mutex>
#include <condition_variable>
#include <array>
#include <cstring>

namespace ps3eye::detail {

struct frame_queue final
{
    explicit frame_queue();
    void init(unsigned frame_size);

    uint8_t* buffer() { return buffer_.data(); }
    uint8_t* enqueue();

    [[nodiscard]]
    bool dequeue(uint8_t* dest, int W, int H, format fmt, bool flip_v);

private:
    static constexpr unsigned max_frame_size = 640*480;
    static constexpr unsigned max_buffered_frames = 50;

    std::mutex mutex_;
    std::condition_variable notify_frame_;
    std::array<uint8_t, max_frame_size * max_buffered_frames> buffer_;

    unsigned size_ = UINT_MAX;
    unsigned head_ = 0;
    unsigned tail_ = 0;
    unsigned available_ = 0;
};

} // ns ps3eye::detail
