#undef NDEBUG
#include <cassert>

#include "mgr.hpp"
#include "internal.hpp"
#include "ps3eye.hpp"

#include <libusb.h>

namespace ps3eye::detail {

extern volatile bool _ps3eye_debug_status;

enum {
    vendor_id = 0x1415,
    product_id = 0x2000,
};

usb_manager::usb_manager()
{
    libusb_init(&usb_context);
    libusb_set_option(usb_context,
                      LIBUSB_OPTION_LOG_LEVEL,
                      _ps3eye_debug_status ? LIBUSB_LOG_LEVEL_INFO : LIBUSB_LOG_LEVEL_NONE);
}

usb_manager::~usb_manager()
{
    //ps3eye_debug("usb_manager destructor");
    if (update_thread.joinable())
        stop_xfer_thread();
    libusb_exit(usb_context);
}

usb_manager& usb_manager::instance()
{
    static usb_manager ret;
    return ret;
}

void usb_manager::camera_started()
{
    assert(usb_context);

    if (active_camera_count.fetch_add(1) == 0)
        start_xfer_thread();
}

void usb_manager::camera_stopped()
{
    if (active_camera_count.fetch_sub(1) == 1)
        stop_xfer_thread();
}

void usb_manager::start_xfer_thread()
{
    update_thread = std::thread(&usb_manager::xfer_callback, this);
}

void usb_manager::stop_xfer_thread()
{
    exit_signaled = true;
    update_thread.join();
    // Reset the exit signal flag.
    // If we don't and we call start_xfer_thread() again, xfer_callback
    // will exit immediately.
    exit_signaled = false;
}

void usb_manager::xfer_callback()
{
    struct timeval tv { 0, 100 * 1000 /* ms */ };

    while (!(exit_signaled.load(std::memory_order_relaxed)))
        libusb_handle_events_timeout_completed(usb_context, &tv, nullptr);
}

std::vector<std::shared_ptr<camera>> usb_manager::list_devices()
{
    std::vector<std::shared_ptr<camera>> list;

    libusb_device* dev;
    libusb_device** devs;
    libusb_device_handle* devhandle;
    int i = 0;
    int cnt;

    cnt = (int)libusb_get_device_list(usb_context, &devs);

    if (cnt < 0)
    {
        ps3eye_debug("Error Device scan");
    }

    cnt = 0;
    while ((dev = devs[i++]) != nullptr)
    {
        struct libusb_device_descriptor desc;
        libusb_get_device_descriptor(dev, &desc);
        if (desc.idVendor == vendor_id && desc.idProduct == product_id)
        {
            int err = libusb_open(dev, &devhandle);
            if (err == 0)
            {
                libusb_close(devhandle);
                list.push_back(std::make_shared<camera>(dev));
                libusb_ref_device(dev);
                cnt++;
            }
            else
                ps3eye_debug("failed open device: %d", err);
        }
    }

    libusb_free_device_list(devs, 1);

    return list;
}
void usb_manager::set_debug(bool value)
{
    if (value == _ps3eye_debug_status)
        return;

    _ps3eye_debug_status = value;
    libusb_set_option(usb_context,
                      LIBUSB_OPTION_LOG_LEVEL,
                      value ? LIBUSB_LOG_LEVEL_INFO : LIBUSB_LOG_LEVEL_NONE);
}

} // ns ps3eye::detail
