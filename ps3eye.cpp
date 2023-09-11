// source code from https://github.com/inspirit/PS3EYEDriver
#include "ps3eye.hpp"
#include "urb.hpp"
#include "mgr.hpp"
#include "internal.hpp"

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>
#include <algorithm>
#include <iterator>
#include <chrono>

#ifdef __clang__
#   pragma clang diagnostic ignored "-Wzero-length-array"
#endif

#ifdef _MSC_VER
#   pragma warning(disable : 4200) // zero-length arrays
#   pragma warning(disable : 4996) // 'This function or variable may be unsafe': snprintf
#endif

#include <libusb.h>

enum : uint8_t {
    OV534_REG_ADDRESS = 0xf1, /* sensor address */
    OV534_REG_SUBADDR = 0xf2,
    OV534_REG_WRITE = 0xf3,
    OV534_REG_READ = 0xf4,
    OV534_REG_OPERATION = 0xf5,
    OV534_REG_STATUS = 0xf6,
    OV534_OP_WRITE_3 = 0x37,
    OV534_OP_WRITE_2 = 0x33,
    OV534_OP_READ_2 = 0xf9,
};

using namespace std::chrono_literals;

namespace ps3eye::detail {
volatile bool _ps3eye_debug_status = true;
} // ns ps3eye::detail

using ps3eye::detail::ps3eye_debug;
using ps3eye::detail::rate_s;

namespace ps3eye {

static const uint8_t ov534_reg_initdata[][2] = {
    { 0xe7, 0x3a },

    { OV534_REG_ADDRESS, 0x42 }, /* select OV772x sensor */

    { 0x92, 0x01 },
    { 0x93, 0x18 },
    { 0x94, 0x10 },
    { 0x95, 0x10 },
    { 0xE2, 0x00 },
    { 0xE7, 0x3E },

    { 0x96, 0x00 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x0A },
    { 0x97, 0x3F },
    { 0x97, 0x4A },
    { 0x97, 0x20 },
    { 0x97, 0x15 },
    { 0x97, 0x0B },

    { 0x8E, 0x40 },
    { 0x1F, 0x81 },
    { 0xC0, 0x50 },
    { 0xC1, 0x3C },
    { 0xC2, 0x01 },
    { 0xC3, 0x01 },
    { 0x50, 0x89 },
    { 0x88, 0x08 },
    { 0x8D, 0x00 },
    { 0x8E, 0x00 },

    { 0x1C, 0x00 }, /* video data start (V_FMT) */

    { 0x1D, 0x00 }, /* RAW8 mode */
    { 0x1D, 0x02 }, /* payload size 0x0200 * 4 = 2048 bytes */
    { 0x1D, 0x00 }, /* payload size */

    { 0x1D, 0x01 }, /* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
    { 0x1D, 0x2C }, /* frame size */
    { 0x1D, 0x00 }, /* frame size */

    { 0x1C, 0x0A }, /* video data start (V_CNTL0) */
    { 0x1D, 0x08 }, /* turn on UVC header */
    { 0x1D, 0x0E },

    { 0x34, 0x05 },
    { 0xE3, 0x04 },
    { 0x89, 0x00 },
    { 0x76, 0x00 },
    { 0xE7, 0x2E },
    { 0x31, 0xF9 },
    { 0x25, 0x42 },
    { 0x21, 0xF0 },
    { 0xE5, 0x04 }
};

static const uint8_t ov772x_reg_initdata[][2] = {

    { 0x12, 0x80 }, /* reset */
    { 0x3D, 0x00 },

    { 0x12, 0x01 },
    /* Processed Bayer RAW (8bit) */
    { 0x11, 0x01 },
    { 0x14, 0x40 },
    { 0x15, 0x00 },
    { 0x63, 0xAA },
    // AWB
    { 0x64, 0x87 },
    { 0x66, 0x00 },
    { 0x67, 0x02 },
    { 0x17, 0x26 },
    { 0x18, 0xA0 },
    { 0x19, 0x07 },
    { 0x1A, 0xF0 },
    { 0x29, 0xA0 },
    { 0x2A, 0x00 },
    { 0x2C, 0xF0 },
    { 0x20, 0x10 },
    { 0x4E, 0x0F },
    { 0x3E, 0xF3 },
    { 0x0D, 0x41 },
    { 0x32, 0x00 },
    { 0x13, 0xF0 },
    // COM8  - jfrancois 0xf0	orig x0f7
    { 0x22, 0x7F },
    { 0x23, 0x03 },
    { 0x24, 0x40 },
    { 0x25, 0x30 },
    { 0x26, 0xA1 },
    { 0x2A, 0x00 },
    { 0x2B, 0x00 },
    { 0x13, 0xF7 },
    { 0x0C, 0xC0 },

    { 0x11, 0x00 },
    { 0x0D, 0x41 },

    { 0x8E, 0x00 },
    // De-noise threshold - jfrancois 0x00 - orig 0x04
};

static const uint8_t bridge_start_vga[][2] = {
    { 0x1c, 0x00 },
    { 0x1d, 0x00 },
    { 0x1d, 0x02 },
    { 0x1d, 0x00 },
    { 0x1d, 0x01 }, /* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
    { 0x1d, 0x2C },
    /* frame size */
    { 0x1d, 0x00 },
    /* frame size */
    { 0xc0, 0x50 },
    { 0xc1, 0x3c },
};
static const uint8_t sensor_start_vga[][2] = {
    { 0x12, 0x01 },
    { 0x17, 0x26 },
    { 0x18, 0xa0 },
    { 0x19, 0x07 },
    { 0x1a, 0xf0 },
    { 0x29, 0xa0 },
    { 0x2c, 0xf0 },
    { 0x65, 0x20 },
};
static const uint8_t bridge_start_qvga[][2] = {
    { 0x1c, 0x00 },
    { 0x1d, 0x00 },
    { 0x1d, 0x02 },
    { 0x1d, 0x00 },
    { 0x1d, 0x00 }, /* frame size = 0x004B00 * 4 = 76800 bytes (320 * 240 @ 8bpp) */
    { 0x1d, 0x4b },
    /* frame size */
    { 0x1d, 0x00 },
    /* frame size */
    { 0xc0, 0x28 },
    { 0xc1, 0x1e },
};
static const uint8_t sensor_start_qvga[][2] = {
    { 0x12, 0x41 },
    { 0x17, 0x3f },
    { 0x18, 0x50 },
    { 0x19, 0x03 },
    { 0x1a, 0x78 },
    { 0x29, 0x50 },
    { 0x2c, 0x78 },
    { 0x65, 0x2f },
};

camera::camera(libusb_device* device) : device_(device)
{
}

camera::~camera()
{
    stop();
    release();
    if (device_)
        libusb_unref_device(device_);
}

void camera::release()
{
    if (handle_)
    {
        stop();
        close_usb();
    }
    handle_ = nullptr;
    set_error(ERROR_OK);
}

void camera::set_error(int code)
{
    if (code == ERROR_OK || error_code_ == ERROR_OK)
    {
        error_code_ = code;
        if (code != ERROR_OK)
            ps3eye_debug("usb error %s (%d)", error_string(), code);
    }
}

bool camera::init(resolution res, int framerate, format fmt)
{
    set_error(ERROR_OK);
    stop();
    if (error_code_ != ERROR_OK)
        release();

    // open usb device so we can setup and go
    if (!handle_ && !open_usb())
        return false;

    resolution_ = res;

    framerate_ = ov534_set_frame_rate(framerate, true);
    format_ = fmt;

    /* reset bridge */
    ov534_reg_write(0xe7, 0x3a);
    ov534_reg_write(0xe0, 0x08);

    std::this_thread::sleep_for(10ms);

    /* initialize the sensor address */
    ov534_reg_write(OV534_REG_ADDRESS, 0x42);

    /* reset sensor */
    sccb_reg_write(0x12, 0x80);

    std::this_thread::sleep_for(10ms);

#if 0
    /* probe the sensor */
    sccb_reg_read(0x0a);
    uint16_t sensor_id = sccb_reg_read(0x0a) << 8;
    sccb_reg_read(0x0b);
    sensor_id |= sccb_reg_read(0x0b);
    ps3eye_debug("Sensor ID: %04x", sensor_id);
#endif

    /* initialize */
    reg_w_array(ov534_reg_initdata, std::size(ov534_reg_initdata));
    ov534_set_led(1);
    sccb_w_array(ov772x_reg_initdata, std::size(ov772x_reg_initdata));
    ov534_reg_write(0xe0, 0x09);
    ov534_set_led(0);

    return true;
}

bool camera::start()
{
    if (!is_initialized() || streaming_ || error_code_ != ERROR_OK)
        return false;

    if (resolution_ == res_QVGA)
    { /* 320x240 */
        reg_w_array(bridge_start_qvga, std::size(bridge_start_qvga));
        sccb_w_array(sensor_start_qvga, std::size(sensor_start_qvga));
    }
    else
    { /* 640x480 */
        reg_w_array(bridge_start_vga, std::size(bridge_start_vga));
        sccb_w_array(sensor_start_vga, std::size(sensor_start_vga));
    }

    ov534_set_frame_rate(framerate_);

    set_hue(hue_);
    set_saturation(saturation_);
    set_awb(awb_);
    set_auto_gain(auto_gain_);
    set_gain(gain_);
    set_exposure(exposure_);
    set_brightness(brightness_);
    set_contrast(contrast_);
    set_sharpness(sharpness_);
    set_red_balance(red_balance_);
    set_blue_balance(blue_balance_);
    set_green_balance(green_balance_);
    set_flip_status(flip_h_, flip_v_);

    ov534_set_led(1);
    ov534_reg_write(0xe0, 0x00); // start stream

    // init and start urb
    auto [ w, h ] = size();
    urb.start_transfers(handle_, unsigned(w * h));
    streaming_ = true;

    return true;
}

void camera::stop()
{
    if (!streaming_)
        return;

    if (handle_)
    {
        /* stop streaming data */
        ov534_reg_write(0xe0, 0x09);
        ov534_set_led(0);

        // close urb
        urb.close_transfers();
    }

    streaming_ = false;
}

#define MAX_USB_DEVICE_PORT_PATH 7

bool camera::usb_port(char* buf, unsigned sz) const
{
    bool success = false;

    if (is_initialized())
    {
        uint8_t port_numbers[MAX_USB_DEVICE_PORT_PATH];

        memset(buf, 0, sz);
        memset(port_numbers, 0, sizeof(port_numbers));

        int cnt = libusb_get_port_numbers(device_, port_numbers, MAX_USB_DEVICE_PORT_PATH);
        int bus_id = libusb_get_bus_number(device_);

        snprintf(buf, sz, "b%d", bus_id);

        if (cnt > 0)
        {
            success = true;

            for (int i = 0; i < cnt; i++)
            {
                uint8_t port_number = port_numbers[i];
                char port_string[8];

                snprintf(port_string, sizeof(port_string),
                         (i == 0) ? "_p%d" : ".%d", port_number);

                if (strlen(buf) + strlen(port_string) + 1 <= sz)
                    std::strcat(buf, port_string);
                else
                {
                    success = false;
                    break;
                }
            }
        }
    }

    return success;
}

int camera::bytes_per_pixel() const
{
    if (format_ == format::Bayer)
        return 1;
    else if (format_ == format::BGR)
        return 3;
    else if (format_ == format::RGB)
        return 3;
	else if (format_ == format::BGRA)
		return 4;
	else if (format_ == format::RGBA)
		return 4;
    else if (format_ == format::Gray)
        return 1;
    return 0;
}

bool camera::get_frame(uint8_t* frame)
{
    if (!streaming_)
        return false;

    if (error_code_ != ERROR_OK && handle_)
    {
        stop();
        release();
        return false;
    }

    auto [ w, h ] = size();
    return urb.queue.dequeue(frame, w, h, format_, flip_v_);
}

bool camera::open_usb()
{
    // open, set first config and claim interface
    int res = libusb_open(device_, &handle_);
    if (res != 0)
    {
        ps3eye_debug("device open error: %d", res);
        set_error(res);
        return false;
    }

    // Linux has a kernel module for the PS3 eye camera (that's where most of
    // the code in here comes from..) so we must detach the driver before we can
    // hook up with the eye ourselves
    //libusb_detach_kernel_driver(handle_, 0);

    // libusb_set_configuration(handle_, 0);

    res = libusb_claim_interface(handle_, 0);
    if (res != 0)
    {
        ps3eye_debug("device claim interface error: %d", res);
        set_error(res);
        return false;
    }

    return true;
}

void camera::close_usb()
{
    libusb_release_interface(handle_, 0);
    libusb_attach_kernel_driver(handle_, 0);
    libusb_close(handle_);
    handle_ = nullptr;
}

/* Two bits control LED: 0x21 bit 7 and 0x23 bit 7.
 * (direction and output)? */
void camera::ov534_set_led(int status)
{
    uint8_t data;

    //ps3eye_debug("led status: %d\n", status);

    data = ov534_reg_read(0x21);
    data |= 0x80;
    ov534_reg_write(0x21, data);

    data = ov534_reg_read(0x23);
    if (status)
        data |= 0x80;
    else
        data &= ~0x80;

    ov534_reg_write(0x23, data);

    if (!status)
    {
        data = ov534_reg_read(0x21);
        data &= ~0x80;
        ov534_reg_write(0x21, data);
    }
}

int camera::normalize_framerate(int fps, resolution res)
{
    return _normalize_framerate(fps, res).fps;
}

const rate_s& camera::_normalize_framerate(int fps, resolution res)
{
    static const struct rate_s rate_0[] = {
        /* 640x480 */
        { 83, 0x01, 0xc1, 0x02 }, /* 83 FPS: video is partly corrupt */
        { 75, 0x01, 0x81, 0x02 }, /* 75 FPS or below: video is valid */
        { 60, 0x00, 0x41, 0x04 },
        { 50, 0x01, 0x41, 0x02 },
        { 40, 0x02, 0xc1, 0x04 },
        { 30, 0x04, 0x81, 0x02 },
        { 15, 0x03, 0x41, 0x04 },
    };

    static const struct rate_s rate_1[] = {
        /* 320x240 */
        { 290, 0x00, 0xc1, 0x04 },
        { 205, 0x01, 0xc1, 0x02 }, /* 205 FPS or above: video is partly corrupt */
        { 187, 0x01, 0x81, 0x02 }, /* 187 FPS or below: video is valid */
        { 150, 0x00, 0x41, 0x04 },
        { 137, 0x02, 0xc1, 0x02 },
        { 125, 0x01, 0x41, 0x02 },
        { 100, 0x02, 0xc1, 0x04 },
        {  90, 0x03, 0x81, 0x02 },
        {  75, 0x04, 0x81, 0x02 },
        {  60, 0x04, 0xc1, 0x04 },
        {  50, 0x04, 0x41, 0x02 },
        {  40, 0x06, 0x81, 0x03 },
        {  37, 0x03, 0x41, 0x04 },
        {  30, 0x04, 0x41, 0x04 },
    };

    struct rate_s const* r;
    int i;

    switch (res)
    {
    default:
    case res_VGA:
        r = rate_0;
        i = std::size(rate_0);
        break;
    case res_QVGA:
        r = rate_1;
        i = std::size(rate_1);
        break;
    }

    while (--i > 0)
    {
        if (fps >= r->fps) break;
        r++;
    }

    return *r;
}

int camera::normalize_framerate(int fps)
{
    return normalize_framerate(fps, resolution_);
}

/* validate frame rate and (if not dry run) set it */
int camera::ov534_set_frame_rate(int frame_rate, bool dry_run)
{
    const struct rate_s& rate = _normalize_framerate(frame_rate, resolution_);

    if (!dry_run)
    {
        sccb_reg_write(0x11, rate.r11);
        sccb_reg_write(0x0d, rate.r0d);
        ov534_reg_write(0xe5, rate.re5);
    }

    return rate.fps;
}

#if defined __clang__ && __clang_major__ >= 10
#   pragma clang diagnostic push
#   pragma clang diagnostic ignored "-Wenum-enum-conversion"
#endif

void camera::ov534_reg_write(uint16_t reg, uint8_t val)
{
    if (error_code_ != ERROR_OK)
        return;

    int ret;

    // debug("reg=0x%04x, val=0%02x", reg, val);
    usb_buf[0] = val;

    ret = libusb_control_transfer(handle_, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                  0x01, 0x00, reg, usb_buf.data(), 1, 500);
    if (ret < 0)
        error_code_ = ret;
}

uint8_t camera::ov534_reg_read(uint16_t reg)
{
    if (error_code_ != ERROR_OK)
        return 0;

    int ret;

    ret = libusb_control_transfer(handle_, LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
                                  0x01, 0x00, reg, usb_buf.data(), 1, 500);

    // debug("reg=0x%04x, data=0x%02x", reg, usb_buf[0]);
    if (ret < 0)
    {
        error_code_ = ret;
        return 0;
    }
    else
        return usb_buf[0];
}

#if defined __clang__ && __clang_major__ >= 10
#   pragma clang diagnostic pop
#endif

bool camera::sccb_check_status()
{
    if (error_code_ != ERROR_OK)
        return false;

    bool ret = false;

    for (int i = 0; i < 5; i++)
    {
        uint8_t data = ov534_reg_read(OV534_REG_STATUS);

        if (error_code_ != ERROR_OK)
            return false;

        switch (data)
        {
        case 0x00:
            ret = true;
            goto end;
        case 0x04:
            ret = false;
            goto end;
        case 0x03:
            break;
        default:
            ps3eye_debug("sccb status 0x%02x, attempt %d/5", data, i + 1);
        }

        std::this_thread::yield();
    }

    ps3eye_debug("sscb status failure");

end:
    return ret;
}

void camera::sccb_reg_write(uint8_t reg, uint8_t val)
{
    // debug("reg: 0x%02x, val: 0x%02x", reg, val);
    ov534_reg_write(OV534_REG_SUBADDR, reg);
    ov534_reg_write(OV534_REG_WRITE, val);
    ov534_reg_write(OV534_REG_OPERATION, OV534_OP_WRITE_3);

    (void)sccb_check_status();
}

uint8_t camera::sccb_reg_read(uint16_t reg)
{
    ov534_reg_write(OV534_REG_SUBADDR, (uint8_t)reg);
    ov534_reg_write(OV534_REG_OPERATION, OV534_OP_WRITE_2);
    (void)sccb_check_status();

    ov534_reg_write(OV534_REG_OPERATION, OV534_OP_READ_2);
    (void)sccb_check_status();

    return ov534_reg_read(OV534_REG_READ);
}
/* output a bridge sequence (reg - val) */
void camera::reg_w_array(const uint8_t (*data)[2], int len)
{
    while (--len >= 0)
    {
        if (error_code_ != ERROR_OK)
            break;
        ov534_reg_write((*data)[0], (*data)[1]);
        data++;
    }
}

/* output a sensor sequence (reg - val) */
void camera::sccb_w_array(const uint8_t (*data)[2], int len)
{
    while (--len >= 0)
    {
        if ((*data)[0] != 0xff)
        {
            sccb_reg_write((*data)[0], (*data)[1]);
        }
        else
        {
            sccb_reg_read((*data)[1]);
            sccb_reg_write(0xff, 0x00);
        }
        data++;
    }
}

const char* camera::error_string() const
{
    if (error_code_ == ERROR_OK)
        return nullptr;

    return libusb_strerror((libusb_error)error_code_);
}

} // namespace ps3eye::detail
