#include <stdio.h>
#include <SDL2/SDL.h>
#undef main

#include "lvgl.h"
#include "lvgl/src/drivers/sdl/lv_sdl_window.h"
#include "lvgl/src/drivers/sdl/lv_sdl_mouse.h"
#include "lvgl/src/drivers/sdl/lv_sdl_keyboard.h"

int main(void)
{
    printf("Starting OSM map test...\n");

    /* Initialize LVGL */
    lv_init();

    /* Create SDL window */
    lv_display_t *display =
        lv_sdl_window_create(800, 600);

    if (display == NULL)
    {
        printf("Failed to create SDL window\n");
        return 1;
    }

    /* Mouse */
    lv_indev_t *mouse =
        lv_sdl_mouse_create();

    (void)mouse;

    /* Keyboard */
    lv_indev_t *keyboard =
        lv_sdl_keyboard_create();

    (void)keyboard;

    /*
     * Create title
     */
    lv_obj_t *title =
        lv_label_create(lv_screen_active());

    lv_label_set_text(
        title,
        "OpenStreetMap Tile Test");

    lv_obj_align(
        title,
        LV_ALIGN_TOP_MID,
        0,
        20);

    /*
     * Create image object
     */
    lv_obj_t *map =
        lv_image_create(lv_screen_active());

    /*
     * Load OSM PNG tile
     */
    lv_image_set_src(
        map,
        "C:/OSM_LVGL_App/map_tiles/test_tile.png");

    /*
     * Put image in center
     */
    lv_obj_align(
        map,
        LV_ALIGN_CENTER,
        0,
        20);

    printf("OSM tile loaded.\n");

    /*
     * LVGL event loop
     */
    while (1)
    {
        lv_timer_handler();

        SDL_Delay(5);
    }

    return 0;
}