#undef NDEBUG
#include "queue.hpp"

#include <chrono>
#include <cassert>

namespace ps3eye::detail {

void frame_queue::init(unsigned frame_size)
{
    size_ = frame_size;
    head_ = 0;
    tail_ = 0;
    available_ = 0;
}

frame_queue::frame_queue() = default;

uint8_t* frame_queue::enqueue()
{
    assert(size_ != UINT_MAX);

    uint8_t* new_frame = nullptr;
    std::lock_guard<std::mutex> lock(mutex_);

    // Unlike traditional producer/consumer, we don't block the producer if
    // the buffer is full (ie. the consumer is not reading data fast
    // enough). Instead, if the buffer is full, we simply return the current
    // frame pointer, causing the producer to overwrite the previous frame.
    // This allows performance to degrade gracefully: if the consumer is not
    // fast enough (< Camera FPS), it will miss frames, but if it is fast
    // enough (>= Camera FPS), it will see everything.
    //
    // Note that because the the producer is writing directly to the ring
    // buffer, we can only ever be a maximum of num_frames-1 ahead of the
    // consumer, otherwise the producer could overwrite the frame the
    // consumer is currently reading (in case of a slow consumer)
    if (available_ >= max_buffered_frames - 1)
    {
        return buffer_.data() + head_ * size_;
    }

    // Note: we don't need to copy any data to the buffer since the USB
    // packets are directly written to the frame buffer. We just need to
    // update head and available count to signal to the consumer that a new
    // frame is available
    head_ = (head_ + 1) % max_buffered_frames;
    available_ = std::min(available_ + 1, max_buffered_frames);

    // Determine the next frame pointer that the producer should write to
    new_frame = buffer_.data() + head_ * size_;

    // Signal consumer that data became available
    notify_frame_.notify_one();

    return new_frame;
}

static void debayer_gray(int W, int H, const uint8_t* __restrict input, uint8_t* __restrict buf)
{
    // PSMove output is in the following Bayer format (GRBG):
    //
    // G R G R G R
    // B G B G B G
    // G R G R G R
    // B G B G B G
    //
    // This is the normal Bayer pattern shifted left one place.

    int source_stride = W;
    const uint8_t* source_row = input; // Start at first bayer pixel
    int dest_stride = W;
    uint8_t* dest_row = buf + dest_stride + 1; // We start outputting
    // at the second pixel
    // of the second row's
    // G component
    uint32_t R, G, B;

    // Fill rows 1 to height-2 of the destination buffer. First and last row
    // are filled separately (they are copied from the second row and
    // second-to-last rows respectively)
    for (int y = 0; y < H - 2;
         source_row += source_stride, dest_row += dest_stride, ++y)
    {
        const uint8_t* source = source_row;
        // -2 to deal with the fact that we're starting at the second pixel of
        // the row and should end at the second-to-last pixel of the row
        // (first and last are filled separately)
        const uint8_t* source_end =
            source + (source_stride - 2);
        uint8_t* dest = dest_row;

        // Row starting with Green
        if (y % 2 == 0)
        {
            // Fill first pixel (green)
            B = (source[source_stride] + source[source_stride + 2] + 1) >> 1;
            G = source[source_stride + 1];
            R = (source[1] + source[source_stride * 2 + 1] + 1) >> 1;
            *dest = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);

            source++;
            dest++;

            // Fill remaining pixel
            for (; source <= source_end - 2; source += 2, dest += 2)
            {
                // Blue pixel
                B = source[source_stride + 1];
                G = (source[1] + source[source_stride] + source[source_stride + 2] +
                     source[source_stride * 2 + 1] + 2) >> 2;
                R = (source[0] + source[2] + source[source_stride * 2] +
                     source[source_stride * 2 + 2] + 2) >> 2;
                dest[0] = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);

                //  Green pixel
                B = (source[source_stride + 1] + source[source_stride + 3] + 1) >> 1;
                G = source[source_stride + 2];
                R = (source[2] + source[source_stride * 2 + 2] + 1) >> 1;
                dest[1] = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);
            }
        }
        else
        {
            for (; source <= source_end - 2; source += 2, dest += 2)
            {
                // Red pixel
                B = (source[0] + source[2] + source[source_stride * 2] +
                     source[source_stride * 2 + 2] + 2) >> 2;

                G = (source[1] + source[source_stride] + source[source_stride + 2] +
                     source[source_stride * 2 + 1] + 2) >> 2;

                R = source[source_stride + 1];
                dest[0] = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);

                // Green pixel
                B = (source[2] + source[source_stride * 2 + 2] + 1) >> 1;
                G = source[source_stride + 2];
                R = (source[source_stride + 1] + source[source_stride + 3] + 1) >> 1;
                dest[1] = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);
            }
        }

        if (source < source_end)
        {
            B = source[source_stride + 1];
            G = (source[1] + source[source_stride] + source[source_stride + 2] +
                 source[source_stride * 2 + 1] + 2) >> 2;
            R = (source[0] + source[2] + source[source_stride * 2] +
                 source[source_stride * 2 + 2] + 2) >> 2;

            dest[0] = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);

            //source++;
            //dest++;
        }

        // Fill first pixel of row (copy second pixel)
        uint8_t* first_pixel = dest_row - 1;
        first_pixel[0] = dest_row[0];

        // Fill last pixel of row (copy second-to-last pixel). Note: dest row
        // starts at the *second* pixel of the row, so dest_row + (width-2)
        // * num_output_channels puts us at the last pixel of the row
        uint8_t* last_pixel = dest_row + (W - 2);
        uint8_t* second_to_last_pixel = last_pixel - 1;
        last_pixel[0] = second_to_last_pixel[0];
    }

    // Fill first & last row
    for (int i = 0; i < dest_stride; i++)
    {
        buf[i] = buf[i + dest_stride];
        buf[i + (H - 1) * dest_stride] = buf[i + (H - 2) * dest_stride];
    }
}

template<int num_channels>
inline static void setAlpha(uint8_t *destGreen);

template<>
inline void detail::setAlpha<3>(uint8_t *destGreen) {}

template<>
inline void detail::setAlpha<4>(uint8_t *destGreen) { destGreen[2] = 255; }

template<int num_output_channels, bool in_BGR, int swap_br = in_BGR ? 1 : -1>
static void debayer_rgb(int W, int H, const uint8_t* __restrict input, uint8_t* __restrict buf)
{
    // PSMove output is in the following Bayer format (GRBG):
    //
    // G R G R G R
    // B G B G B G
    // G R G R G R
    // B G B G B G
    //
    // This is the normal Bayer pattern shifted left one place.

    int source_stride = W;
    const uint8_t* source_row = input; // Start at first bayer pixel
    int dest_stride = W * num_output_channels;
    // We start outputting at the second pixel of the
    uint8_t* dest_row = buf + dest_stride + num_output_channels + 1;
    // second row's G component

    // Fill rows 1 to height-2 of the destination buffer. First and last row
    // are filled separately (they are copied from the second row and
    // second-to-last rows respectively)
    for (int y = 0; y < H - 2;
         source_row += source_stride, dest_row += dest_stride, ++y)
    {
        const uint8_t* source = source_row;
        // -2 to deal with the fact that we're starting at the second pixel of
        // the row and should end at the second-to-last pixel of the row
        // (first and last are filled separately)
        const uint8_t* source_end = source + (source_stride - 2);
        uint8_t* dest = dest_row;

        // Row starting with Green
        if (y % 2 == 0)
        {
            // Fill first pixel (green)
            dest[-1 * swap_br] =
                (source[source_stride] + source[source_stride + 2] + 1) >> 1;
            dest[0] = source[source_stride + 1];
            dest[1 * swap_br] = (source[1] + source[source_stride * 2 + 1] + 1) >> 1;
			setAlpha<num_output_channels>(dest);

            source++;
            dest += num_output_channels;

            // Fill remaining pixel
            for (; source <= source_end - 2; source += 2, dest += num_output_channels * 2)
            {
                // Blue pixel
                uint8_t* cur_pixel = dest;
                cur_pixel[-1 * swap_br] = source[source_stride + 1];
                cur_pixel[0] = (source[1] + source[source_stride] +
                                source[source_stride + 2] +
                                source[source_stride * 2 + 1] + 2) >> 2;
                cur_pixel[1 * swap_br] =
                    (source[0] + source[2] + source[source_stride * 2] +
                     source[source_stride * 2 + 2] + 2) >> 2;
				setAlpha<num_output_channels>(cur_pixel);

                //  Green pixel
                uint8_t* next_pixel = cur_pixel + num_output_channels;
                next_pixel[-1 * swap_br] =
                    (source[source_stride + 1] + source[source_stride + 3] + 1) >> 1;
                next_pixel[0] = source[source_stride + 2];
                next_pixel[1 * swap_br] =
                    (source[2] + source[source_stride * 2 + 2] + 1) >> 1;
				setAlpha<num_output_channels>(next_pixel);
            }
        }
        else
        {
            for (; source <= source_end - 2; source += 2, dest += num_output_channels * 2)
            {
                // Red pixel
                uint8_t* cur_pixel = dest;
                cur_pixel[-1 * swap_br] =
                    (source[0] + source[2] + source[source_stride * 2] +
                     source[source_stride * 2 + 2] + 2) >> 2;

                cur_pixel[0] =
                    (source[1] + source[source_stride] +
                     source[source_stride + 2] +
                     source[source_stride * 2 + 1] + 2) >> 2;

                cur_pixel[1 * swap_br] = source[source_stride + 1];
				setAlpha<num_output_channels>(cur_pixel);

                // Green pixel
                uint8_t* next_pixel = cur_pixel + num_output_channels;
                next_pixel[-1 * swap_br] =
                    (source[2] + source[source_stride * 2 + 2] + 1) >> 1;
                next_pixel[0] = source[source_stride + 2];
                next_pixel[1 * swap_br] =
                    (source[source_stride + 1] + source[source_stride + 3] + 1) >> 1;
				setAlpha<num_output_channels>(next_pixel);
            }
        }

        if (source < source_end)
        {
            dest[-1 * swap_br] = source[source_stride + 1];
            dest[0] = (source[1] + source[source_stride] + source[source_stride + 2] +
                       source[source_stride * 2 + 1] + 2) >> 2;
            dest[1 * swap_br] = (source[0] + source[2] + source[source_stride * 2] +
                                 source[source_stride * 2 + 2] + 2) >> 2;
			setAlpha<num_output_channels>(dest);

            //source++;
            //dest += num_output_channels;
        }

        // Fill first pixel of row (copy second pixel)
        uint8_t* first_pixel = dest_row - num_output_channels;
        first_pixel[-1 * swap_br] = dest_row[-1 * swap_br];
        first_pixel[0] = dest_row[0];
        first_pixel[1 * swap_br] = dest_row[1 * swap_br];
		setAlpha<num_output_channels>(first_pixel);

        // Fill last pixel of row (copy second-to-last pixel). Note: dest row
        // starts at the *second* pixel of the row, so dest_row + (width-2)
        // * num_output_channels puts us at the last pixel of the row
        uint8_t* last_pixel = dest_row + (W - 2) * num_output_channels;
        uint8_t* second_to_last_pixel = last_pixel - num_output_channels;

        last_pixel[-1 * swap_br] = second_to_last_pixel[-1 * swap_br];
        last_pixel[0] = second_to_last_pixel[0];
        last_pixel[1 * swap_br] = second_to_last_pixel[1 * swap_br];
		setAlpha<num_output_channels>(last_pixel);
    }

    // Fill first & last row
    for (int i = 0; i < dest_stride; i++)
    {
        buf[i] = buf[i + dest_stride];
        buf[i + (H - 1) * dest_stride] = buf[i + (H - 2) * dest_stride];
    }
}

bool frame_queue::dequeue(uint8_t* dest, int W, int H, format fmt)
{
    assert(size_ != UINT_MAX);

    std::unique_lock<std::mutex> lock(mutex_);
    using namespace std::chrono_literals;

    // If there is no data in the buffer, wait until data becomes available
    bool status = notify_frame_.wait_for(lock, 1s, [this]() { return available_ != 0; });
    if (!status)
        return false;

    // Copy from internal buffer
    uint8_t* source = buffer_.data() + size_ * tail_;

    switch (fmt)
    {
    case format::Bayer:
        memcpy(dest, source, size_);
        break;
    case format::BGR:
        debayer_rgb<3, true>(W, H, source, dest);
        break;
    case format::RGB:
        debayer_rgb<3, false>(W, H, source, dest);
        break;
	case format::BGRA:
		debayer_rgb<4, true>(W, H, source, dest);
		break;
	case format::RGBA:
		debayer_rgb<4, false>(W, H, source, dest);
		break;
    case format::Gray:
        debayer_gray(W, H, source, dest);
        break;
    default:
        ps3eye_debug("invalid format %d in dequeue()", (int)fmt);
        break;
    }

    // Update tail and available count
    tail_ = (tail_ + 1) % max_buffered_frames;
    available_--;

    return true;
}

} // ns ps3eye::detail
