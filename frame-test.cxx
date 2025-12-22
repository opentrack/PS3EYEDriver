#include "ps3eye.hpp"

int main(void)
{
    ps3eye::camera::set_debug(true);

    auto cameras = ps3eye::list_devices();

    if (cameras.empty())
    {
        fprintf(stderr, "no device\n");
        return 1;
    }

    std::vector<uint8_t> vec;
    auto camera = cameras[0];

    bool running = true;
    running &= camera->init(ps3eye::res_VGA, 75, ps3eye::fmt_BGR);
    running &= camera->start();

    if (!running)
    {
        fprintf(stderr, "device init failed\n");
        goto fail;
    }

    vec.resize(unsigned(camera->stride() * camera->height()));

    for (;;)
        if (!camera->get_frame((uint8_t*)vec.data()))
        {
            fprintf(stderr, "can't get frame: %s (%d)\n", camera->error_string(), camera->error_code());
            goto fail;
        }

fail:
    return 1;
}

