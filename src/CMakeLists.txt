cmake_minimum_required(VERSION 3.20)

project(OSM_LVGL_App C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)

pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(CJSON REQUIRED libcjson)
pkg_check_modules(CURL REQUIRED libcurl)

set(LV_CONF_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lv_conf.h)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../lvgl
    lvgl_build
)

add_executable(OSM_LVGL_App
    main.c
)

target_include_directories(OSM_LVGL_App PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../lvgl
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${SDL2_INCLUDE_DIRS}
    ${CJSON_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
)

target_link_directories(OSM_LVGL_App PRIVATE
    ${SDL2_LIBRARY_DIRS}
    ${CJSON_LIBRARY_DIRS}
    ${CURL_LIBRARY_DIRS}
)

target_link_libraries(OSM_LVGL_App PRIVATE
    lvgl
    ${SDL2_LIBRARIES}
    ${CJSON_LIBRARIES}
    ${CURL_LIBRARIES}
    png
)