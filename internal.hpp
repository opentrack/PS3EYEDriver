#pragma once

#ifdef PS3EYE_DEBUG
#   include <cstdio>
#endif

namespace ps3eye::detail {

extern volatile bool _ps3eye_debug_status;

#ifdef PS3EYE_DEBUG

#ifdef __GNUG__
#   pragma GCC diagnostic push
#   pragma GCC diagnostic ignored "-Wformat-security"
#   pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif

template<unsigned N, typename... xs>
inline void ps3eye_debug(const char(&fmt)[N], const xs&... args)
{
    if (_ps3eye_debug_status)
    {
        fputs("ps3eye: ", stderr);
        fprintf(stderr, fmt, args...);
        fputs("\n", stderr);
    }
}

#ifdef __GNUG__
#   pragma GCC diagnostic pop
#endif

#else
template<unsigned N, typename... xs>
inline void ps3eye_debug(const char(&)[N], const xs&... args)
{
}
#endif

} // ns ps3eye::detail

namespace ps3eye {
enum class format
{
    Bayer, // Output in Bayer. Destination buffer must be width * height bytes
    BGR, // Output in BGR. Destination buffer must be width * height * 3 bytes
    RGB, // Output in RGB. Destination buffer must be width * height * 3 bytes
	BGRA,// Output in BGRA. Destination buffer must be width * height * 4 bytes
	RGBA,// Output in RGBA. Destination buffer must be width * height * 4 bytes
    Gray // Output in Grayscale. Destination buffer must be width * height bytes
};
} // ns ps3eye
