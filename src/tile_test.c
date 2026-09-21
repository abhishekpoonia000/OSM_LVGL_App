#include <stdio.h>
#include <math.h>

#define ZOOM 15

void latlon_to_tile(
    double latitude,
    double longitude,
    int zoom,
    int *tile_x,
    int *tile_y)
{
    double lat_rad;
    double n;

    n = pow(2.0, zoom);

    lat_rad =
        latitude * M_PI / 180.0;

    *tile_x =
        (int)((longitude + 180.0) / 360.0 * n);

    *tile_y =
        (int)(
            (1.0 -
             asinh(tan(lat_rad)) / M_PI)
            / 2.0
            * n
        );
}

int main(void)
{
    double latitude;
    double longitude;

    int tile_x;
    int tile_y;

    /*
     * Bangalore coordinates
     */

    latitude = 12.9716;
    longitude = 77.5946;

    latlon_to_tile(
        latitude,
        longitude,
        ZOOM,
        &tile_x,
        &tile_y);

    printf("\n");
    printf("==============================\n");
    printf("OSM TILE CALCULATION\n");
    printf("==============================\n");

    printf(
        "Latitude  : %.6f\n",
        latitude);

    printf(
        "Longitude : %.6f\n",
        longitude);

    printf(
        "Zoom      : %d\n",
        ZOOM);

    printf(
        "Tile X    : %d\n",
        tile_x);

    printf(
        "Tile Y    : %d\n",
        tile_y);

    printf("==============================\n");

    return 0;
}