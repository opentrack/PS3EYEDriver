#include "ps3eye.hpp"
#include <thread>
#include <chrono>
#include <cstdlib>

static bool test(ps3eye::camera& cam, int fps, ps3eye::resolution res)
{
    std::vector<uint8_t> buf;

    if (!cam.init(res, fps)) { fprintf(stderr, "failed to init camera\n"); return false; }
    if (!cam.start()) { fprintf(stderr, "failed to start camera\n"); return false; }

    auto [ w, h ] = cam.size();
    auto size = unsigned(w*h*cam.bytes_per_pixel());
    buf.resize(size);

    bool ret = false;

    using namespace std::chrono_literals;

    for (unsigned k = 0; k < 10; k++)
    {
        if (fps > 0)
            std::this_thread::sleep_for(1ms * 1000/fps);

        ret |= cam.get_frame(buf.data());
        if (ret)
            break;
    }

    if (ret)
    {
        for (int k = 0; k < 5; k++)
        {
            if (fps > 0)
                std::this_thread::sleep_for(1ms * 750/fps);

            ret &= cam.get_frame(buf.data());
            if (!ret)
            {
                ps3eye::detail::ps3eye_debug("read failed on frame %d", k);
                break;
            }
        }
    }
    else
        ps3eye::detail::ps3eye_debug("can't read any frame");

    printf("[%s] %dx%d@%dHz\n", ret ? "GOOD" : "FAIL", cam.width(), cam.height(), cam.framerate());

    return ret;
}

static bool iter_modes(ps3eye::camera& cam, ps3eye::resolution res)
{
    int last_fps = -1;
    bool status = true;
    for (int x = 0; x < 256; x++)
    {
        int fps = ps3eye::camera::normalize_framerate(x, res);
        if (fps == last_fps)
            continue;
        last_fps = fps;
        status &= test(cam, fps, res);
    }
    return status;
}

int main(void)
{
    ps3eye::camera::set_debug(true);
    auto devices = ps3eye::list_devices();
    constexpr int ex_NOINPUT = 66 /* sysexits(3) */;

    if (devices.empty())
    {
        fprintf(stderr, "no camera\n");
        return ex_NOINPUT;
    }

    auto cam = devices[0];
    bool status = true;

    status &= iter_modes(*cam, ps3eye::res_QVGA);
    status &= iter_modes(*cam, ps3eye::res_VGA);

    ps3eye::camera::set_debug(false);

    return status ? EXIT_SUCCESS : ex_NOINPUT;
}
