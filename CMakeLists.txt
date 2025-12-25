set(CMAKE_CXX_VISIBILITY_PRESET "hidden")

add_definitions(-DNOMINMAX)
add_definitions(-DWIN32_LEAN_AND_MEAN)
add_definitions(-D_CRT_SECURE_NO_WARNINGS)

add_definitions(-DPS3EYE_DEBUG)

if(NOT WIN32)
    add_definitions(-pthread)
    link_libraries(-pthread)
endif()

if(NOT MSVC)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(libusb "libusb-1.0" QUIET IMPORTED_TARGET)
    endif()
    if(libusb_FOUND)
        link_libraries(PkgConfig::libusb)
    endif()
else()
    set(SDK_LIBUSB CACHE PATH "")
    if(SDK_LIBUSB)
        set(libusb_FOUND TRUE)
        link_directories("${SDK_LIBUSB}")
        include_directories(SYSTEM "${SDK_LIBUSB}")
        link_libraries(libusb-1.0)
    endif()
endif()

if(libusb_FOUND)
    add_library(ps3eye-driver STATIC
        "ps3eye.cpp"
        "urb.cpp"
        "queue.cpp"
        "mgr.cpp"
        "accessors.cpp"
    )
endif()

if(TARGET ps3eye-driver)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
    find_package(SDL2 QUIET)

    if(SDL2_FOUND)
        include_directories(SYSTEM ${SDL2_INCLUDE_DIRS})
        link_libraries(${SDL2_LIBRARIES})
        link_libraries(ps3eye-driver)
        add_executable(ps3eye-sdl "sdl.cxx")
    endif()

    add_executable(ps3eye-mode-test "mode-test.cxx")
    target_link_libraries(ps3eye-mode-test ps3eye-driver)

    add_executable(ps3eye-frame-test "frame-test.cxx")
    target_link_libraries(ps3eye-frame-test ps3eye-driver)
endif()
