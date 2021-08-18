#include "ps3eye.hpp"
#include "mgr.hpp"

using ps3eye::detail::usb_manager;
using ps3eye::detail::_ps3eye_debug_status;
using ps3eye::detail::ps3eye_debug;

namespace ps3eye {

void camera::set_auto_gain(bool val)
{
    auto_gain_ = val;
    constexpr int mask = 1 << 0 /* AEC */ | 1 << 2 /* AGC */;
    if (val)
    {
        sccb_reg_write(0x13, sccb_reg_read(0x13) | mask);
        sccb_reg_write(0x64, sccb_reg_read(0x64) | 0x03);
    }
    else
    {
        sccb_reg_write(0x13, sccb_reg_read(0x13) & ~mask);
        sccb_reg_write(0x64, sccb_reg_read(0x64) & ~0x03);

        set_gain(gain_);
        set_exposure(exposure_);
    }
}

void camera::set_awb(bool val)
{
    awb_ = val;

    if (val)
    {
        sccb_reg_write(0x13, sccb_reg_read(0x13) | 0x02);
        sccb_reg_write(0x63, sccb_reg_read(0x63) | 0xc0);
    }
    else
    {
        sccb_reg_write(0x13, sccb_reg_read(0x13) & ~0x02);
        sccb_reg_write(0x63, sccb_reg_read(0x63) & ~0xc0);

        set_red_balance(red_balance_);
        set_green_balance(green_balance_);
        set_blue_balance(blue_balance_);
    }
}

void camera::set_framerate(int val)
{
    if (!streaming_)
        framerate_ = normalize_framerate(val);
    else
        ps3eye_debug("Can't change framerate while streaming");
}

void camera::set_test_pattern_status(bool enable)
{
    test_pattern_ = enable;
    uint8_t val = sccb_reg_read(0x0C);
    val &= ~0b00000001;
    if (test_pattern_) val |= 0b00000001; // 0x80;
    sccb_reg_write(0x0C, val);
}

void camera::set_exposure(int val)
{
    exposure_ = val;
    sccb_reg_write(0x08, exposure_ >> 7);
    sccb_reg_write(0x10, uint8_t(exposure_ << 1));
}

void camera::set_sharpness(int val)
{
    sharpness_ = val;
    sccb_reg_write(0x91, sharpness_); // vga noise
    sccb_reg_write(0x8E, sharpness_); // qvga noise
}

void camera::set_contrast(int val)
{
    contrast_ = val;
    sccb_reg_write(0x9C, contrast_);
}

void camera::set_brightness(int val)
{
    brightness_ = val;
    sccb_reg_write(0x9B, brightness_);
}

void camera::set_hue(int val)
{
    hue_ = val;
    sccb_reg_write(0x01, (uint8_t)hue_);
}

void camera::set_red_balance(int val)
{
    red_balance_ = val;
    sccb_reg_write(0x43, red_balance_);
}

void camera::set_blue_balance(int val)
{
    blue_balance_ = val;
    sccb_reg_write(0x42, blue_balance_);
}

void camera::set_green_balance(int val)
{
    green_balance_ = val;
    sccb_reg_write(0x44, green_balance_);
}

void camera::set_flip_status(bool horizontal, bool vertical)
{
    flip_h_ = horizontal;
    flip_v_ = vertical;
    uint8_t val = sccb_reg_read(0x0c);
    val &= ~0xc0;
    if (!horizontal) val |= 0x40;
    if (!vertical) val |= 0x80;
    sccb_reg_write(0x0c, val);
}

void camera::set_gain(int val)
{
    gain_ = val;
    val = gain_;
    switch (val & 0x30)
    {
    case 0x00:
        val &= 0x0F;
        break;
    case 0x10:
        val &= 0x0F;
        val |= 0x30;
        break;
    case 0x20:
        val &= 0x0F;
        val |= 0x70;
        break;
    case 0x30:
        val &= 0x0F;
        val |= 0xF0;
        break;
    }
    sccb_reg_write(0x00, (uint8_t)val);
}

void camera::set_saturation(int val)
{
    saturation_ = val;
    sccb_reg_write(0xa7, saturation_); /* U saturation */
    sccb_reg_write(0xa8, saturation_); /* V saturation */
}

void camera::set_debug(bool value)
{
    usb_manager::instance().set_debug(value);
    _ps3eye_debug_status = value;
}

std::pair<int, int> camera::size() const
{
    switch (resolution_)
    {
    default:
    case res_VGA:
        return { 640, 480 };
    case res_QVGA:
        return { 320, 240 };
    }
}

std::vector<std::shared_ptr<camera>> list_devices()
{
    return usb_manager::instance().list_devices();
}

} // ns ps3eye
