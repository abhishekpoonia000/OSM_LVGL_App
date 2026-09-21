#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <pthread.h>

#include "lvgl.h"
#include "lvgl/src/drivers/sdl/lv_sdl_window.h"
#include "lvgl/src/drivers/sdl/lv_sdl_mouse.h"
#include "lvgl/src/drivers/sdl/lv_sdl_keyboard.h"

#include <cjson/cJSON.h>
#include <png.h>
#include <curl/curl.h>

/* ============================================================
 * OSM + LVGL NAVIGATION SIMULATOR
 *
 * Features:
 *   - START = current GPS position
 *   - DESTINATION = Nominatim search
 *   - Search any place, not only India
 *   - Exact place names from OSM display_name
 *   - Destination suggestions
 *   - Reverse geocoding for map clicks
 *   - OSM map tiles
 *   - Exact source/destination marker positions
 *   - Map labels for START and DESTINATION
 *   - Pan and zoom
 *
 * Current GPS is simulated. Replace gps_get_current() later
 * with the real vehicle GPS/GNSS wrapper.
 * ============================================================ */

#define WINDOW_WIDTH  800
#define WINDOW_HEIGHT 600

#define TILE_SIZE     256
#define MAP_IMAGE_SIZE 1024

/* Full map viewport below the top search bar. */
#define VIEW_X        0
#define VIEW_Y        0
#define VIEW_W        800
#define VIEW_H        600

#define MAX_RESULTS   5
#define MAX_URL       4096
#define MAX_RESPONSE  (2 * 1024 * 1024)
#define MAX_ROUTE_POINTS 16384
#define NAME_SIZE     512

#define MIN_SEARCH_CHARS 2

#define DEFAULT_ZOOM 13
#define MIN_ZOOM     3
#define MAX_ZOOM     18

#define EARTH_RADIUS_KM 6371.0

/* ============================================================
 * LOCATION
 * ============================================================ */

typedef struct
{
    char name[NAME_SIZE];
    double lat;
    double lon;
    double distance_km;
} Location;

static Location results[MAX_RESULTS];
static int result_count = 0;

/* ============================================================
 * GPS - SIMULATED FOR NOW
 * ============================================================ */

static double current_lat = 12.971600;
static double current_lon = 77.594600;
static int gps_available = 1;


static int gps_get_current(double *lat, double *lon)
{
    if (!gps_available || !lat || !lon)
        return 0;

    *lat = current_lat;
    *lon = current_lon;
    return 1;
}

/* ============================================================
 * DESTINATION
 * ============================================================ */

static double destination_lat = 0.0;
static double destination_lon = 0.0;
static int destination_valid = 0;
static char destination_name[NAME_SIZE] = "";

/* ============================================================
 * LVGL OBJECTS
 * ============================================================ */

static lv_obj_t *search_box = NULL;
static lv_obj_t *results_box = NULL;
 static lv_obj_t *destination_button = NULL;
static lv_obj_t *start_info = NULL;
static lv_obj_t *destination_info = NULL;

static lv_obj_t *map_view = NULL;
static lv_obj_t *map_image = NULL;
static lv_obj_t *map_search_box = NULL;
static lv_obj_t *map_search_button = NULL;
static lv_obj_t *map_suggestions_box = NULL;

static lv_obj_t *current_marker = NULL;
static lv_obj_t *destination_marker = NULL;
static lv_obj_t *current_name_label = NULL;
static lv_obj_t *destination_name_label = NULL;


static lv_obj_t *zoom_in_button = NULL;
static lv_obj_t *zoom_out_button = NULL;
static lv_obj_t *my_location_button = NULL;
static lv_obj_t *back_button = NULL;
static lv_obj_t *map_start_button = NULL;
static lv_obj_t *map_attribution = NULL;
static lv_obj_t *route_line = NULL;
static lv_point_precise_t *route_draw_points = NULL;

/* Dedicated navigation screen objects. */
static lv_obj_t *navigation_back_button = NULL;
static lv_obj_t *navigation_stop_button = NULL;
static lv_obj_t *navigation_title = NULL;
static lv_obj_t *navigation_instruction = NULL;
static lv_obj_t *navigation_destination_label = NULL;
static lv_obj_t *navigation_distance_label = NULL;
static lv_obj_t *navigation_eta_label = NULL;

/* ============================================================
 * KEYBOARD / TIMERS
 * ============================================================ */

static lv_indev_t *keyboard_indev = NULL;
static lv_group_t *keyboard_group = NULL;

/* Debounced search-as-you-type state. Photon is used for suggestions because
 * the public Nominatim service does not support client-side autocomplete. */
static lv_timer_t *suggestion_timer = NULL;
static lv_timer_t *suggestion_poll_timer = NULL;
static lv_obj_t *active_suggestion_box = NULL;
static lv_obj_t *active_search_input = NULL;
static pthread_mutex_t suggestion_mutex = PTHREAD_MUTEX_INITIALIZER;
static int suggestion_busy = 0;

/* Give every Photon worker its own response file. */
static pthread_mutex_t photon_file_mutex = PTHREAD_MUTEX_INITIALIZER;
static unsigned long photon_file_counter = 0;
static unsigned long suggestion_generation = 0;
static Location pending_suggestions[MAX_RESULTS];
static int pending_suggestion_count = 0;
static unsigned long pending_generation = 0;
static char pending_query[512] = "";
static int pending_ready = 0;

typedef struct
{
    char query[512];
    unsigned long generation;
} SuggestionJob;

/* Defer destination selection until the current LVGL click callback returns. */
typedef struct
{
    double lat;
    double lon;
    char name[NAME_SIZE];
} DeferredSelection;

static DeferredSelection deferred_selection;
static int deferred_selection_pending = 0;

/* ============================================================
 * MAP IMAGE DATA
 * ============================================================ */

static lv_image_dsc_t map_dsc;
static uint8_t *map_pixels = NULL;

static double map_center_lat = 12.971600;
static double map_center_lon = 77.594600;
static int map_zoom = DEFAULT_ZOOM;
static int map_visual_scale = 256;

/* Route from START/current GPS to the searched destination. */
typedef struct
{
    double lat;
    double lon;
} RoutePoint;

static RoutePoint route_points[MAX_ROUTE_POINTS];
static int route_point_count = 0;
static int route_valid = 0;
static double route_distance_m = 0.0;
static double route_duration_s = 0.0;
static double route_cumulative_m[MAX_ROUTE_POINTS];

static lv_timer_t *navigation_timer = NULL;
static int navigation_active = 0;
static int navigation_route_index = 0;
static double navigation_segment_progress_m = 0.0;
static double navigation_speed_mps = 13.9;
static double navigation_total_distance_m = 0.0;
static double navigation_total_duration_s = 0.0;
static int navigation_follow_mode = 1;

/* 0 = search screen, 1 = map screen, 2 = navigation screen. */
static int app_page = 0;

/* ============================================================
 * DRAG STATE
 * ============================================================ */

static int dragging = 0;
static int drag_moved = 0;
static lv_point_t drag_start;

/* ============================================================
 * PROTOTYPES
 * ============================================================ */

static void create_search_screen(void);
static void show_map(void);
static void rebuild_map(void);
static void update_all_map_labels(void);
static void update_markers(void);
static void clear_route_line(void);
static void draw_route_line(void);
static int request_route(void);
static void map_start_cb(lv_event_t *e);
static void show_navigation_screen(void);
static void navigation_timer_cb(lv_timer_t *timer);
static void stop_navigation_simulation(void);
static void navigation_stop_cb(lv_event_t *e);
static void map_navigation_button_cb(lv_event_t *e);
static void perform_search(const char *query);

static void search_button_event_cb(lv_event_t *e);
 static void destination_button_cb(lv_event_t *e);
static void result_button_cb(lv_event_t *e);
static void map_search_button_cb(lv_event_t *e);
static void search_input_value_changed_cb(lv_event_t *e);
static void suggestion_timer_cb(lv_timer_t *timer);
static void suggestion_poll_cb(lv_timer_t *timer);
static void start_suggestion_search(lv_obj_t *input, lv_obj_t *box);
static int photon_search(const char *query, Location *out, int *out_count);
static void map_press_cb(lv_event_t *e);
static void map_pressing_cb(lv_event_t *e);
static void map_release_cb(lv_event_t *e);
static void zoom_in_cb(lv_event_t *e);
static void zoom_out_cb(lv_event_t *e);
static void my_location_cb(lv_event_t *e);
static void back_cb(lv_event_t *e);
static void cancel_active_suggestions(void);
static void deferred_select_destination_cb(void *param);
static void select_destination(double lat, double lon, const char *name);
static void select_destination_on_existing_map(double lat, double lon, const char *name);

static void latlon_to_world(double lat, double lon, int zoom,
                            double *x, double *y);
static void world_to_latlon(double x, double y, int zoom,
                            double *lat, double *lon);
static void map_pixel_position(double lat, double lon, int *sx, int *sy);

/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

static void safe_copy(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0)
        return;

    if (!src)
        src = "";

    snprintf(dst, dst_size, "%s", src);
}

static void trim_text(char *text)
{
    if (!text)
        return;

    size_t len = strlen(text);

    while (len > 0 && isspace((unsigned char)text[len - 1]))
        text[--len] = '\0';

    size_t start = 0;
    while (text[start] && isspace((unsigned char)text[start]))
        start++;

    if (start > 0)
        memmove(text, text + start, strlen(text + start) + 1);
}

static void short_place_name(const char *full, char *out, size_t out_size)
{
    if (!out || out_size == 0)
        return;

    out[0] = '\0';

    if (!full)
        return;

    /* Show the actual place name, i.e. the first part of display_name. */
    const char *comma = strchr(full, ',');
    size_t len = comma ? (size_t)(comma - full) : strlen(full);

    while (len > 0 && isspace((unsigned char)full[len - 1]))
        len--;

    if (len == 0)
        len = strlen(full);

    if (len >= out_size)
        len = out_size - 1;

    memcpy(out, full, len);
    out[len] = '\0';
}

static double haversine_km(double lat1, double lon1,
                           double lat2, double lon2)
{
    double p1 = lat1 * M_PI / 180.0;
    double p2 = lat2 * M_PI / 180.0;
    double dp = (lat2 - lat1) * M_PI / 180.0;
    double dl = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dp / 2.0) * sin(dp / 2.0) +
               cos(p1) * cos(p2) *
               sin(dl / 2.0) * sin(dl / 2.0);

    if (a > 1.0)
        a = 1.0;

    return EARTH_RADIUS_KM * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

static void url_encode(const char *in, char *out, size_t size)
{
    size_t j = 0;

    if (!in || !out || size == 0)
        return;

    for (size_t i = 0; in[i] && j + 1 < size; i++)
    {
        unsigned char c = (unsigned char)in[i];

        if ((c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~')
        {
            out[j++] = (char)c;
        }
        else if (c == ' ')
        {
            out[j++] = '+';
        }
        else if (j + 3 < size)
        {
            snprintf(out + j, size - j, "%%%02X", c);
            j += 3;
        }
    }

    out[j] = '\0';
}

/* ============================================================
 * CURL / FILE HELPERS
 * ============================================================
 */

static const char *temp_directory(void)
{
#ifdef _WIN32
    const char *p = getenv("TEMP");
    return (p && *p) ? p : ".";
#else
    return "/tmp";
#endif
}

static size_t curl_file_write_callback(void *contents,
                                        size_t size,
                                        size_t nmemb,
                                        void *userp)
{
    FILE *file = (FILE *)userp;
    return fwrite(contents, size, nmemb, file);
}

static int run_curl_to_file(const char *url, const char *file_path,
                            int timeout_seconds)
{
    CURL *curl = curl_easy_init();
    if (!curl)
    {
        printf("HTTP ERROR: libcurl initialization failed.\n");
        return 0;
    }

    FILE *file = fopen(file_path, "wb");
    if (!file)
    {
        printf("HTTP ERROR: cannot open response file: %s\n", file_path);
        curl_easy_cleanup(curl);
        return 0;
    }

    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Accept: */*");
    headers = curl_slist_append(headers, "Accept-Language: en");

    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, (long)timeout_seconds);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "OSM-LVGL-App/1.0");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_file_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, file);

    printf("HTTP REQUEST: %s\n", url);

    CURLcode rc = curl_easy_perform(curl);

    fclose(file);
    curl_slist_free_all(headers);

    if (rc != CURLE_OK)
    {
        printf("HTTP ERROR: %s\n", curl_easy_strerror(rc));
        curl_easy_cleanup(curl);
        remove(file_path);
        return 0;
    }

    long response_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);

    printf("HTTP RESPONSE: %ld\n", response_code);

    curl_easy_cleanup(curl);
    return (response_code >= 200 && response_code < 300);
}

static char *read_entire_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return NULL;

    if (fseek(f, 0, SEEK_END) != 0)
    {
        fclose(f);
        return NULL;
    }

    long size = ftell(f);
    if (size <= 0 || size >= MAX_RESPONSE)
    {
        fclose(f);
        return NULL;
    }

    rewind(f);

    char *data = (char *)malloc((size_t)size + 1);
    if (!data)
    {
        fclose(f);
        return NULL;
    }

    size_t n = fread(data, 1, (size_t)size, f);
    fclose(f);

    data[n] = '\0';
    return data;
}

/* ============================================================
 * SEARCH - NOMINATIM
 * ============================================================ */

static int photon_search(const char *query, Location *out, int *out_count)
{
    if (!query || !out || !out_count)
        return 0;

    *out_count = 0;

    char clean_query[512];
    safe_copy(clean_query, sizeof(clean_query), query);
    trim_text(clean_query);

    if (strlen(clean_query) < MIN_SEARCH_CHARS)
        return 0;

    char encoded[2048];
    url_encode(clean_query, encoded, sizeof(encoded));

    /*
     * Photon is an OpenStreetMap-based geocoder and explicitly supports
     * search-as-you-type. We also bias results toward the simulated/current
     * GPS position so nearby places are ranked usefully.
     */
    char url[MAX_URL];
    snprintf(url, sizeof(url),
             "https://photon.komoot.io/api/?q=%s&limit=%d&lang=en"
             "&lat=%.8f&lon=%.8f",
             encoded, MAX_RESULTS, current_lat, current_lon);

    /* Search-as-you-type may have multiple HTTP workers at once.
     * Never let them share the same JSON response file. */
    unsigned long file_id;
    pthread_mutex_lock(&photon_file_mutex);
    file_id = ++photon_file_counter;
    pthread_mutex_unlock(&photon_file_mutex);

    char response_file[1024];
#ifdef _WIN32
    snprintf(response_file, sizeof(response_file),
             "%s\\osm_photon_search_%lu.json",
             temp_directory(), file_id);
#else
    snprintf(response_file, sizeof(response_file),
             "%s/osm_photon_search_%lu.json",
             temp_directory(), file_id);
#endif

    if (!run_curl_to_file(url, response_file, 10))
    {
        printf("ERROR: Photon search failed.\\n");
        return 0;
    }

    char *json = read_entire_file(response_file);
    if (!json)
    {
        printf("ERROR: Cannot read Photon response.\\n");
        return 0;
    }

    cJSON *root = cJSON_Parse(json);
    free(json);

    if (!root || !cJSON_IsObject(root))
    {
        if (root)
            cJSON_Delete(root);
        printf("ERROR: Invalid Photon JSON.\\n");
        return 0;
    }

    cJSON *features = cJSON_GetObjectItemCaseSensitive(root, "features");
    if (!cJSON_IsArray(features))
    {
        cJSON_Delete(root);
        return 0;
    }

    cJSON *feature = NULL;
    cJSON_ArrayForEach(feature, features)
    {
        if (*out_count >= MAX_RESULTS)
            break;

        cJSON *geometry = cJSON_GetObjectItemCaseSensitive(feature, "geometry");
        cJSON *properties = cJSON_GetObjectItemCaseSensitive(feature, "properties");

        if (!cJSON_IsObject(geometry) || !cJSON_IsObject(properties))
            continue;

        cJSON *coordinates = cJSON_GetObjectItemCaseSensitive(geometry, "coordinates");
        if (!cJSON_IsArray(coordinates) || cJSON_GetArraySize(coordinates) < 2)
            continue;

        cJSON *lon_item = cJSON_GetArrayItem(coordinates, 0);
        cJSON *lat_item = cJSON_GetArrayItem(coordinates, 1);
        if (!cJSON_IsNumber(lon_item) || !cJSON_IsNumber(lat_item))
            continue;

        char display[NAME_SIZE] = "";
        const char *name = NULL;
        const char *street = NULL;
        const char *city = NULL;
        const char *state = NULL;
        const char *country = NULL;

        cJSON *v = cJSON_GetObjectItemCaseSensitive(properties, "name");
        if (cJSON_IsString(v)) name = v->valuestring;
        v = cJSON_GetObjectItemCaseSensitive(properties, "street");
        if (cJSON_IsString(v)) street = v->valuestring;
        v = cJSON_GetObjectItemCaseSensitive(properties, "city");
        if (cJSON_IsString(v)) city = v->valuestring;
        if (!city)
        {
            v = cJSON_GetObjectItemCaseSensitive(properties, "district");
            if (cJSON_IsString(v)) city = v->valuestring;
        }
        v = cJSON_GetObjectItemCaseSensitive(properties, "state");
        if (cJSON_IsString(v)) state = v->valuestring;
        v = cJSON_GetObjectItemCaseSensitive(properties, "country");
        if (cJSON_IsString(v)) country = v->valuestring;

        if (name && *name)
            safe_copy(display, sizeof(display), name);
        else if (street && *street)
            safe_copy(display, sizeof(display), street);
        else
            safe_copy(display, sizeof(display), clean_query);

        if (city && *city && strcmp(city, display) != 0)
        {
            strncat(display, ", ", sizeof(display) - strlen(display) - 1);
            strncat(display, city, sizeof(display) - strlen(display) - 1);
        }
        if (state && *state && strcmp(state, display) != 0)
        {
            strncat(display, ", ", sizeof(display) - strlen(display) - 1);
            strncat(display, state, sizeof(display) - strlen(display) - 1);
        }
        if (country && *country && strcmp(country, display) != 0)
        {
            strncat(display, ", ", sizeof(display) - strlen(display) - 1);
            strncat(display, country, sizeof(display) - strlen(display) - 1);
        }

        Location *r = &out[*out_count];
        safe_copy(r->name, sizeof(r->name), display);
        r->lat = lat_item->valuedouble;
        r->lon = lon_item->valuedouble;
        r->distance_km = haversine_km(current_lat, current_lon,
                                      r->lat, r->lon);

        printf("Photon result %d: %s\\n", *out_count + 1, r->name);
        printf("                  %.8f, %.8f\\n", r->lat, r->lon);
        (*out_count)++;
    }

    cJSON_Delete(root);
    return *out_count;
}

static int search_places(const char *query)
{
    result_count = 0;
    return photon_search(query, results, &result_count);
}

/* ============================================================
 * REVERSE GEOCODING
 * ============================================================
 */

static int reverse_geocode(double lat, double lon,
                           char *name, size_t name_size)
{
    if (!name || name_size == 0)
        return 0;

    name[0] = '\0';

    char url[MAX_URL];
    snprintf(url, sizeof(url),
             "https://nominatim.openstreetmap.org/reverse"
             "?lat=%.8f&lon=%.8f"
             "&format=json"
             "&zoom=18"
             "&addressdetails=1"
             "&accept-language=en",
             lat, lon);

    char response_file[1024];
    snprintf(response_file, sizeof(response_file),
             "%s\\osm_nominatim_reverse.json", temp_directory());
#ifndef _WIN32
    snprintf(response_file, sizeof(response_file),
             "%s/osm_nominatim_reverse.json", temp_directory());
#endif

    if (!run_curl_to_file(url, response_file, 15))
        return 0;

    char *json = read_entire_file(response_file);
    if (!json)
        return 0;

    cJSON *root = cJSON_Parse(json);
    free(json);

    if (!root || !cJSON_IsObject(root))
    {
        if (root)
            cJSON_Delete(root);
        return 0;
    }

    cJSON *display = cJSON_GetObjectItemCaseSensitive(root, "display_name");

    if (cJSON_IsString(display) && display->valuestring[0])
    {
        safe_copy(name, name_size, display->valuestring);
        cJSON_Delete(root);
        return 1;
    }

    cJSON *address = cJSON_GetObjectItemCaseSensitive(root, "address");
    if (cJSON_IsObject(address))
    {
        const char *road = NULL;
        const char *city = NULL;
        const char *state = NULL;
        const char *country = NULL;

        cJSON *v = cJSON_GetObjectItemCaseSensitive(address, "road");
        if (cJSON_IsString(v)) road = v->valuestring;

        v = cJSON_GetObjectItemCaseSensitive(address, "city");
        if (cJSON_IsString(v)) city = v->valuestring;

        if (!city)
        {
            v = cJSON_GetObjectItemCaseSensitive(address, "town");
            if (cJSON_IsString(v)) city = v->valuestring;
        }

        if (!city)
        {
            v = cJSON_GetObjectItemCaseSensitive(address, "village");
            if (cJSON_IsString(v)) city = v->valuestring;
        }

        v = cJSON_GetObjectItemCaseSensitive(address, "state");
        if (cJSON_IsString(v)) state = v->valuestring;

        v = cJSON_GetObjectItemCaseSensitive(address, "country");
        if (cJSON_IsString(v)) country = v->valuestring;

        name[0] = '\0';
        if (road && *road)
            safe_copy(name, name_size, road);
        if (city && *city)
        {
            if (name[0]) strncat(name, ", ", name_size - strlen(name) - 1);
            strncat(name, city, name_size - strlen(name) - 1);
        }
        if (state && *state)
        {
            if (name[0]) strncat(name, ", ", name_size - strlen(name) - 1);
            strncat(name, state, name_size - strlen(name) - 1);
        }
        if (country && *country)
        {
            if (name[0]) strncat(name, ", ", name_size - strlen(name) - 1);
            strncat(name, country, name_size - strlen(name) - 1);
        }

        trim_text(name);
    }

    cJSON_Delete(root);
    return name[0] != '\0';
}

static void clear_suggestion_box(lv_obj_t *box)
{
    if (box)
        lv_obj_clean(box);
}

static void show_suggestions_in_box(lv_obj_t *box, int count, const char *query)
{
    if (!box)
        return;

    lv_obj_clear_flag(box, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clean(box);

    if (count <= 0)
    {
        lv_obj_t *label = lv_label_create(box);
        lv_label_set_text(label, "No suggestions");
        lv_obj_set_style_pad_all(label, 10, 0);
        return;
    }

    for (int i = 0; i < count; ++i)
    {
        lv_obj_t *button = lv_button_create(box);
        lv_obj_set_width(button, lv_pct(100));
        lv_obj_set_height(button, 58);
        lv_obj_set_style_margin_bottom(button, 4, 0);
        lv_obj_add_event_cb(button, result_button_cb,
                            LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *label = lv_label_create(button);
        char text[800];
        snprintf(text, sizeof(text), "%s\n%.2f km from START",
                 results[i].name, results[i].distance_km);
        lv_label_set_text(label, text);
        lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
        lv_obj_set_width(label, 650);
        lv_obj_center(label);
    }

    (void)query;
}

static void suggestion_poll_cb(lv_timer_t *timer)
{
    (void)timer;

    Location local_results[MAX_RESULTS];
    int local_count = 0;
    unsigned long generation = 0;
    unsigned long current_generation = 0;
    char query[512] = "";
    int ready = 0;

    /* Runs on the LVGL/UI thread. The network worker never calls LVGL. */
    pthread_mutex_lock(&suggestion_mutex);

    if (pending_ready)
    {
        ready = 1;
        local_count = pending_suggestion_count;
        if (local_count > MAX_RESULTS)
            local_count = MAX_RESULTS;

        memcpy(local_results, pending_suggestions,
               sizeof(Location) * (size_t)local_count);
        generation = pending_generation;
        safe_copy(query, sizeof(query), pending_query);

        pending_ready = 0;
        suggestion_busy = 0;
    }

    current_generation = suggestion_generation;
    pthread_mutex_unlock(&suggestion_mutex);

    if (!ready)
        return;

    /* Ignore a response for text that is no longer current. */
    if (generation != current_generation)
    {
        if (active_search_input && active_suggestion_box)
        {
            const char *now = lv_textarea_get_text(active_search_input);
            if (now && strlen(now) >= MIN_SEARCH_CHARS &&
                suggestion_timer == NULL)
            {
                suggestion_timer = lv_timer_create(
                    suggestion_timer_cb, 450, NULL);
            }
        }
        return;
    }

    result_count = local_count;
    memcpy(results, local_results,
           sizeof(Location) * (size_t)local_count);

    if (active_suggestion_box)
    {
        show_suggestions_in_box(active_suggestion_box,
                                 local_count, query);
    }
}

static void *suggestion_worker(void *arg)
{
    SuggestionJob *job = (SuggestionJob *)arg;
    Location local[MAX_RESULTS];
    int count = 0;

    if (!job)
        return NULL;

    printf("\nSUGGESTION SEARCH: [%s]\n", job->query);
    photon_search(job->query, local, &count);

    pthread_mutex_lock(&suggestion_mutex);

    /* Discard stale worker results. Only the newest search may publish. */
    if (job->generation == suggestion_generation)
    {
        pending_suggestion_count = count;
        if (count > 0)
        {
            memcpy(pending_suggestions, local,
                   sizeof(Location) * (size_t)count);
        }
        pending_generation = job->generation;
        safe_copy(pending_query, sizeof(pending_query), job->query);
        pending_ready = 1;
    }

    pthread_mutex_unlock(&suggestion_mutex);

    free(job);
    return NULL;
}

static void start_suggestion_search(lv_obj_t *input, lv_obj_t *box)
{
    if (!input || !box)
        return;

    const char *text = lv_textarea_get_text(input);
    if (!text)
        return;

    char query[512];
    safe_copy(query, sizeof(query), text);
    trim_text(query);

    active_search_input = input;
    active_suggestion_box = box;

    if (strlen(query) < MIN_SEARCH_CHARS)
    {
        clear_suggestion_box(box);
        lv_obj_add_flag(box, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    pthread_mutex_lock(&suggestion_mutex);

    /*
     * Do not block a new search just because an older HTTP request is still
     * running.  Every worker carries its generation number.  The LVGL poll
     * callback accepts only the newest generation, so searches such as
     * Bangalore -> Mysore -> Hassan can overlap safely.
     */
    unsigned long generation = suggestion_generation;
    suggestion_busy = 1;

    pthread_mutex_unlock(&suggestion_mutex);

    SuggestionJob *job = (SuggestionJob *)malloc(sizeof(SuggestionJob));
    if (!job)
    {
        pthread_mutex_lock(&suggestion_mutex);
        suggestion_busy = 0;
        pthread_mutex_unlock(&suggestion_mutex);
        return;
    }

    safe_copy(job->query, sizeof(job->query), query);
    job->generation = generation;

    pthread_t thread;
    if (pthread_create(&thread, NULL, suggestion_worker, job) != 0)
    {
        free(job);
        pthread_mutex_lock(&suggestion_mutex);
        suggestion_busy = 0;
        pthread_mutex_unlock(&suggestion_mutex);
        return;
    }

    pthread_detach(thread);
}

static void suggestion_timer_cb(lv_timer_t *timer)
{
    suggestion_timer = NULL;
    if (!active_search_input || !active_suggestion_box)
        return;

    lv_timer_del(timer);
    start_suggestion_search(active_search_input, active_suggestion_box);
}

static void schedule_suggestions(lv_obj_t *input, lv_obj_t *box)
{
    active_search_input = input;
    active_suggestion_box = box;

    if (suggestion_timer)
    {
        lv_timer_del(suggestion_timer);
        suggestion_timer = NULL;
    }

    suggestion_timer = lv_timer_create(suggestion_timer_cb, 450, NULL);
}

static void search_input_value_changed_cb(lv_event_t *e)
{
    lv_obj_t *input = lv_event_get_target(e);
    if (!input)
        return;

    lv_obj_t *box = (input == map_search_box)
                        ? map_suggestions_box
                        : results_box;
    if (!box)
        return;

    const char *text = lv_textarea_get_text(input);
    if (!text)
        text = "";

    pthread_mutex_lock(&suggestion_mutex);
    suggestion_generation++;
    pthread_mutex_unlock(&suggestion_mutex);

    if (strlen(text) < MIN_SEARCH_CHARS)
    {
        if (suggestion_timer)
        {
            lv_timer_del(suggestion_timer);
            suggestion_timer = NULL;
        }

        clear_suggestion_box(box);
        lv_obj_add_flag(box, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_clear_flag(box, LV_OBJ_FLAG_HIDDEN);
    schedule_suggestions(input, box);
}

/* ============================================================
 * SEARCH SCREEN UI
 * ============================================================
 */

static void clear_results(void)
{
    if (results_box)
        lv_obj_clean(results_box);
}

static void show_search_message(const char *text)
{
    clear_results();

    if (!results_box)
        return;

    lv_obj_t *label = lv_label_create(results_box);
    lv_label_set_text(label, text ? text : "");
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, 690);
    lv_obj_set_style_pad_all(label, 12, 0);
}

static void show_search_results(void)
{
    clear_results();

    if (result_count <= 0)
    {
        show_search_message("No matching place found. Try another name.");
        return;
    }

    for (int i = 0; i < result_count; i++)
    {
        lv_obj_t *button = lv_button_create(results_box);
        lv_obj_set_width(button, 720);
        lv_obj_set_height(button, 68);
        lv_obj_set_style_margin_bottom(button, 8, 0);

        lv_obj_add_event_cb(button, result_button_cb,
                            LV_EVENT_CLICKED, (void *)(intptr_t)i);

        lv_obj_t *label = lv_label_create(button);
        char text[800];

        snprintf(text, sizeof(text),
                 "%d. %s\n%.2f km from START",
                 i + 1, results[i].name, results[i].distance_km);

        lv_label_set_text(label, text);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label, 690);
        lv_obj_center(label);
    }
}

static void search_button_event_cb(lv_event_t *e)
{
    (void)e;

    if (!search_box || !results_box)
        return;

    const char *text = lv_textarea_get_text(search_box);
    if (!text)
        return;

    char query[512];
    safe_copy(query, sizeof(query), text);
    trim_text(query);

    if (strlen(query) < MIN_SEARCH_CHARS)
    {
        show_search_message("Type at least 2 characters.");
        return;
    }

    /*
     * IMPORTANT: Never call search_places()/curl directly from an LVGL
     * button callback. The HTTP request runs in the suggestion worker.
     * This keeps the SDL/LVGL window alive even after repeated searches.
     */
    attach_keyboard_to_textarea(search_box);

    pthread_mutex_lock(&suggestion_mutex);
    suggestion_generation++;
    pending_ready = 0;
    pthread_mutex_unlock(&suggestion_mutex);

    if (suggestion_timer)
    {
        lv_timer_del(suggestion_timer);
        suggestion_timer = NULL;
    }

    clear_results();
    show_search_message("Searching OpenStreetMap...");

    active_search_input = search_box;
    active_suggestion_box = results_box;
    lv_obj_clear_flag(results_box, LV_OBJ_FLAG_HIDDEN);

    /* Start network work outside the LVGL event callback. */
    start_suggestion_search(search_box, results_box);
}

static void perform_search(const char *query)
{
    if (!query)
        return;

    char clean[512];
    safe_copy(clean, sizeof(clean), query);
    trim_text(clean);

    printf("\n============================================\n");
    printf("USER SEARCH: [%s]\n", clean);
    printf("============================================\n");

    result_count = search_places(clean);

    if (result_count == 0)
    {
        show_search_message(
            "No places found. Check the name or Internet connection.");
        printf("SEARCH RESULT: 0 places\n");
        return;
    }

    printf("SEARCH RESULT: %d places\n", result_count);
    show_search_results();
}

/* ============================================================
 * KEYBOARD
 * ============================================================
 */

static void attach_keyboard_to_textarea(lv_obj_t *textarea)
{
    if (!keyboard_indev || !textarea)
        return;

    if (keyboard_group)
    {
        lv_group_delete(keyboard_group);
        keyboard_group = NULL;
    }

    keyboard_group = lv_group_create();
    lv_group_add_obj(keyboard_group, textarea);
    lv_group_focus_obj(textarea);
    lv_indev_set_group(keyboard_indev, keyboard_group);
}

static void style_search_box(lv_obj_t *box)
{
    lv_obj_set_style_radius(box, 22, 0);
    lv_obj_set_style_border_width(box, 1, 0);
    lv_obj_set_style_border_color(box, lv_color_hex(0xDADCE0), 0);
    lv_obj_set_style_bg_color(box, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_left(box, 18, 0);
    lv_obj_set_style_pad_right(box, 18, 0);
    lv_obj_set_style_text_color(box, lv_color_hex(0x202124), 0);
}

/* ============================================================
 * WEB MERCATOR
 * ============================================================
 */

static void latlon_to_world(double lat, double lon, int zoom,
                            double *x, double *y)
{
    double n = pow(2.0, zoom);

    if (lat > 85.05112878) lat = 85.05112878;
    if (lat < -85.05112878) lat = -85.05112878;

    *x = (lon + 180.0) / 360.0 * n * TILE_SIZE;

    double rad = lat * M_PI / 180.0;
    *y = (1.0 - log(tan(rad) + 1.0 / cos(rad)) / M_PI) /
         2.0 * n * TILE_SIZE;
}

static void world_to_latlon(double x, double y, int zoom,
                            double *lat, double *lon)
{
    double world = (double)TILE_SIZE * (double)(1 << zoom);

    while (x < 0) x += world;
    while (x >= world) x -= world;

    if (y < 0) y = 0;
    if (y > world) y = world;

    double n = pow(2.0, zoom);

    *lon = x / (n * TILE_SIZE) * 360.0 - 180.0;

    double yy = M_PI - 2.0 * M_PI * y / (n * TILE_SIZE);
    *lat = 180.0 / M_PI * atan(sinh(yy));
}

/* ============================================================
 * TILE DOWNLOAD / PNG DECODE
 * ============================================================
 */

static int download_tile(int zoom, int tx, int ty, uint8_t *rgba_out)
{
    int side = 1 << zoom;

    while (tx < 0) tx += side;
    while (tx >= side) tx -= side;

    if (ty < 0 || ty >= side)
        return 0;

    char file_path[1024];
    char url[512];

#ifdef _WIN32
    snprintf(file_path, sizeof(file_path),
             "%s\\osm_tile_%d_%d_%d.png",
             temp_directory(), zoom, tx, ty);
#else
    snprintf(file_path, sizeof(file_path),
             "%s/osm_tile_%d_%d_%d.png",
             temp_directory(), zoom, tx, ty);
#endif

    snprintf(url, sizeof(url),
             "https://tile.openstreetmap.org/%d/%d/%d.png",
             zoom, tx, ty);

    FILE *cached = fopen(file_path, "rb");
    if (cached)
    {
        fclose(cached);
    }
    else
    {
        if (!run_curl_to_file(url, file_path, 10))
            return 0;
    }

    FILE *f = fopen(file_path, "rb");
    if (!f)
        return 0;

    if (fseek(f, 0, SEEK_END) != 0)
    {
        fclose(f);
        return 0;
    }

    long size = ftell(f);
    if (size <= 0 || size > 4 * 1024 * 1024)
    {
        fclose(f);
        return 0;
    }

    rewind(f);

    uint8_t *png_data = (uint8_t *)malloc((size_t)size);
    if (!png_data)
    {
        fclose(f);
        return 0;
    }

    size_t n = fread(png_data, 1, (size_t)size, f);
    fclose(f);

    if (n != (size_t)size)
    {
        free(png_data);
        return 0;
    }

    png_image image;
    memset(&image, 0, sizeof(image));
    image.version = PNG_IMAGE_VERSION;

    if (!png_image_begin_read_from_memory(&image,
                                          png_data,
                                          (size_t)size))
    {
        free(png_data);
        return 0;
    }

    image.format = PNG_FORMAT_RGBA;

    int ok = png_image_finish_read(&image, NULL, rgba_out,
                                   0, NULL);

    png_image_free(&image);
    free(png_data);

    return ok ? 1 : 0;
}

static void free_map_image(void)
{
    if (map_pixels)
        free(map_pixels);

    map_pixels = NULL;
    memset(&map_dsc, 0, sizeof(map_dsc));
}

/* ============================================================
 * BUILD MAP IMAGE
 * ============================================================
 */

static int build_map_image(void)
{
    double center_x, center_y;
    latlon_to_world(map_center_lat, map_center_lon, map_zoom,
                    &center_x, &center_y);

    free_map_image();

    map_pixels = (uint8_t *)malloc(
        (size_t)MAP_IMAGE_SIZE * MAP_IMAGE_SIZE * 4);

    if (!map_pixels)
        return 0;

    memset(map_pixels, 0xFF,
           (size_t)MAP_IMAGE_SIZE * MAP_IMAGE_SIZE * 4);

    double start_x = center_x - MAP_IMAGE_SIZE / 2.0;
    double start_y = center_y - MAP_IMAGE_SIZE / 2.0;

    uint8_t *tiles[4][4] = {{0}};

    int tile_x0 = (int)floor(start_x / TILE_SIZE);
    int tile_y0 = (int)floor(start_y / TILE_SIZE);
    int world_tiles = 1 << map_zoom;

    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            int tx = tile_x0 + col;
            int ty = tile_y0 + row;

            if (ty < 0 || ty >= world_tiles)
                continue;

            int wx = tx;
            while (wx < 0) wx += world_tiles;
            while (wx >= world_tiles) wx -= world_tiles;

            tiles[row][col] = (uint8_t *)malloc(
                TILE_SIZE * TILE_SIZE * 4);

            if (!tiles[row][col])
                continue;

            if (!download_tile(map_zoom, wx, ty, tiles[row][col]))
            {
                free(tiles[row][col]);
                tiles[row][col] = NULL;
            }
        }
    }

    for (int y = 0; y < MAP_IMAGE_SIZE; y++)
    {
        double wy = start_y + y;
        int tile_y = (int)floor(wy / TILE_SIZE);
        int local_y = (int)floor(wy) - tile_y * TILE_SIZE;
        int row = tile_y - tile_y0;

        while (local_y < 0) local_y += TILE_SIZE;
        while (local_y >= TILE_SIZE) local_y -= TILE_SIZE;

        if (row < 0 || row >= 4)
            continue;

        for (int x = 0; x < MAP_IMAGE_SIZE; x++)
        {
            double wx = start_x + x;
            int tile_x = (int)floor(wx / TILE_SIZE);
            int local_x = (int)floor(wx) - tile_x * TILE_SIZE;
            int col = tile_x - tile_x0;

            while (local_x < 0) local_x += TILE_SIZE;
            while (local_x >= TILE_SIZE) local_x -= TILE_SIZE;

            if (col < 0 || col >= 4)
                continue;

            uint8_t *tile = tiles[row][col];
            if (!tile)
                continue;

            uint8_t *src = tile +
                ((local_y * TILE_SIZE + local_x) * 4);
            uint8_t *dst = map_pixels +
                ((y * MAP_IMAGE_SIZE + x) * 4);

            memcpy(dst, src, 4);
        }
    }

    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
            free(tiles[row][col]);

    map_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    map_dsc.header.cf = LV_COLOR_FORMAT_ARGB8888;
    map_dsc.header.w = MAP_IMAGE_SIZE;
    map_dsc.header.h = MAP_IMAGE_SIZE;
    map_dsc.header.stride = MAP_IMAGE_SIZE * 4;
    map_dsc.data_size = (uint32_t)MAP_IMAGE_SIZE * MAP_IMAGE_SIZE * 4;
    map_dsc.data = map_pixels;

    return 1;
}

/* ============================================================
 * EXACT MAP COORDINATES
 * ============================================================
 */

static void map_pixel_position(double lat, double lon,
                               int *sx, int *sy)
{
    double px, py, cx, cy;

    latlon_to_world(lat, lon, map_zoom, &px, &py);
    latlon_to_world(map_center_lat, map_center_lon, map_zoom, &cx, &cy);

    double world = (double)TILE_SIZE * (double)(1 << map_zoom);
    double dx = px - cx;

    /* Shortest horizontal path across the world. */
    if (dx > world / 2.0) dx -= world;
    if (dx < -world / 2.0) dx += world;

    double dy = py - cy;
    double scale = (double)map_visual_scale / 256.0;

    /* Return coordinates relative to map_view. */
    *sx = VIEW_W / 2 + (int)round(dx * scale);
    *sy = VIEW_H / 2 + (int)round(dy * scale);
}

/* ============================================================
 * MAP MARKERS
 * ============================================================
 */

static lv_obj_t *create_current_marker(lv_obj_t *parent)
{
    lv_obj_t *marker = lv_obj_create(parent);
    lv_obj_remove_style_all(marker);
    lv_obj_set_size(marker, 34, 34);
    lv_obj_clear_flag(marker, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(marker, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *halo = lv_obj_create(marker);
    lv_obj_remove_style_all(halo);
    lv_obj_set_size(halo, 34, 34);
    lv_obj_set_style_radius(halo, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(halo, lv_color_hex(0x4285F4), 0);
    lv_obj_set_style_bg_opa(halo, LV_OPA_20, 0);
    lv_obj_center(halo);

    lv_obj_t *ring = lv_obj_create(marker);
    lv_obj_remove_style_all(ring);
    lv_obj_set_size(ring, 23, 23);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(ring, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(ring, LV_OPA_COVER, 0);
    lv_obj_center(ring);

    lv_obj_t *dot = lv_obj_create(ring);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, 14, 14);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(0x1A73E8), 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_center(dot);

    return marker;
}

static lv_obj_t *create_destination_marker(lv_obj_t *parent)
{
    lv_obj_t *pin = lv_obj_create(parent);
    lv_obj_remove_style_all(pin);
    lv_obj_set_size(pin, 32, 42);
    lv_obj_clear_flag(pin, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(pin, LV_OBJ_FLAG_EVENT_BUBBLE);

    lv_obj_t *circle = lv_obj_create(pin);
    lv_obj_remove_style_all(circle);
    lv_obj_set_size(circle, 26, 26);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(circle, lv_color_hex(0xEA4335), 0);
    lv_obj_set_style_bg_opa(circle, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(circle, 4, 0);
    lv_obj_set_style_border_color(circle, lv_color_white(), 0);
    lv_obj_align(circle, LV_ALIGN_TOP_MID, 0, 0);

    /* Small stem. */
    lv_obj_t *stem = lv_obj_create(pin);
    lv_obj_remove_style_all(stem);
    lv_obj_set_size(stem, 8, 18);
    lv_obj_set_style_radius(stem, 4, 0);
    lv_obj_set_style_bg_color(stem, lv_color_hex(0xEA4335), 0);
    lv_obj_set_style_bg_opa(stem, LV_OPA_COVER, 0);
    lv_obj_align(stem, LV_ALIGN_TOP_MID, 0, 22);

    return pin;
}

static void position_current_marker(void)
{
    if (!current_marker)
        return;

    int x, y;
    map_pixel_position(current_lat, current_lon, &x, &y);
    lv_obj_set_pos(current_marker, x - 17, y - 17);
}

static void position_destination_marker(void)
{
    if (!destination_marker || !destination_valid)
        return;

    int x, y;
    map_pixel_position(destination_lat, destination_lon, &x, &y);

    /* The bottom of the pin is the exact coordinate. */
    lv_obj_set_pos(destination_marker, x - 16, y - 42);
}

static void position_map_name_label(lv_obj_t *label,
                                    double lat, double lon,
                                    int width)
{
    if (!label)
        return;

    int x, y;
    map_pixel_position(lat, lon, &x, &y);

    int left = x + 16;
    int top = y - 18;

    if (left + width > VIEW_W - 5)
        left = x - width - 16;

    if (left < 5)
        left = 5;

    if (top < 5)
        top = 5;

    if (top > VIEW_H - 40)
        top = VIEW_H - 40;

    lv_obj_set_pos(label, left, top);
}

static void update_markers(void)
{
    position_current_marker();
    position_destination_marker();

    if (current_name_label)
        position_map_name_label(current_name_label,
                                current_lat, current_lon, 170);

    if (destination_name_label && destination_valid)
        position_map_name_label(destination_name_label,
                                destination_lat, destination_lon, 260);
}

/* ============================================================
 * MAP SIDE INFORMATION
 * ============================================================
 */

static void update_all_map_labels(void)
{
    update_markers();
}

/* ============================================================
 * FIT START + DESTINATION IN VIEW
 * ============================================================
 */

static void fit_start_destination(void)
{
    if (!destination_valid)
    {
        map_center_lat = current_lat;
        map_center_lon = current_lon;
        map_zoom = DEFAULT_ZOOM;
        map_visual_scale = 256;
        return;
    }

    double sx, sy, dx, dy;
    latlon_to_world(current_lat, current_lon, DEFAULT_ZOOM, &sx, &sy);
    latlon_to_world(destination_lat, destination_lon, DEFAULT_ZOOM, &dx, &dy);

    double world = (double)TILE_SIZE * (double)(1 << DEFAULT_ZOOM);
    double ddx = dx - sx;

    if (ddx > world / 2.0) dx -= world;
    if (ddx < -world / 2.0) dx += world;

    double min_x = fmin(sx, dx);
    double max_x = fmax(sx, dx);
    double min_y = fmin(sy, dy);
    double max_y = fmax(sy, dy);

    int z = MAX_ZOOM;

    /* Required space is intentionally smaller than viewport so markers
       and labels have some breathing room. */
    while (z > MIN_ZOOM)
    {
        double factor = pow(2.0, z - DEFAULT_ZOOM);
        double w = (max_x - min_x) * factor;
        double h = (max_y - min_y) * factor;

        if (w <= VIEW_W - 110 && h <= VIEW_H - 130)
            break;

        z--;
    }

    double factor = pow(2.0, z - DEFAULT_ZOOM);
    double center_x = ((min_x + max_x) / 2.0) * factor;
    double center_y = ((min_y + max_y) / 2.0) * factor;

    world_to_latlon(center_x, center_y, z,
                    &map_center_lat, &map_center_lon);

    map_zoom = z;
    map_visual_scale = 256;
}

/* ============================================================
 * ROUTE LINE / OSRM ROUTING
 * ============================================================
 */

static void clear_route_line(void)
{
    if (route_line)
    {
        lv_obj_delete(route_line);
        route_line = NULL;
    }

    free(route_draw_points);
    route_draw_points = NULL;
}

static int request_route(void)
{
    double start_lat, start_lon;

    if (!destination_valid)
        return 0;

    if (!gps_get_current(&start_lat, &start_lon))
    {
        printf("ROUTE: GPS position unavailable.\n");
        return 0;
    }

    char url[MAX_URL];
    snprintf(url, sizeof(url),
             "https://router.project-osrm.org/route/v1/driving/"
             "%.8f,%.8f;%.8f,%.8f?overview=full&geometries=geojson",
             start_lon, start_lat,
             destination_lon, destination_lat);

#ifdef _WIN32
    const char *route_file = "%s\\osm_route.json";
#else
    const char *route_file = "%s/osm_route.json";
#endif

    char response_file[1024];
    snprintf(response_file, sizeof(response_file), route_file,
             temp_directory());

    printf("ROUTE REQUEST: %s\n", url);

    if (!run_curl_to_file(url, response_file, 20))
    {
        printf("ROUTE: OSRM request failed.\n");
        return 0;
    }

    FILE *f = fopen(response_file, "rb");
    if (!f)
    {
        printf("ROUTE: Could not open response file.\n");
        return 0;
    }

    if (fseek(f, 0, SEEK_END) != 0)
    {
        fclose(f);
        return 0;
    }

    long file_size = ftell(f);
    if (file_size <= 0 || file_size >= MAX_RESPONSE)
    {
        fclose(f);
        printf("ROUTE: Invalid response size.\n");
        return 0;
    }

    rewind(f);

    char *json = malloc((size_t)file_size + 1);
    if (!json)
    {
        fclose(f);
        return 0;
    }

    size_t got = fread(json, 1, (size_t)file_size, f);
    fclose(f);
    json[got] = '\0';

    cJSON *root = cJSON_Parse(json);
    free(json);

    if (!root)
    {
        printf("ROUTE: Invalid JSON response.\n");
        return 0;
    }

    cJSON *code = cJSON_GetObjectItem(root, "code");
    if (!cJSON_IsString(code) || strcmp(code->valuestring, "Ok") != 0)
    {
        printf("ROUTE: OSRM returned code: %s\n",
               cJSON_IsString(code) ? code->valuestring : "unknown");
        cJSON_Delete(root);
        return 0;
    }

    cJSON *routes = cJSON_GetObjectItem(root, "routes");
    cJSON *route0 = cJSON_IsArray(routes) ? cJSON_GetArrayItem(routes, 0) : NULL;
    cJSON *geometry = route0 ? cJSON_GetObjectItem(route0, "geometry") : NULL;
    cJSON *coordinates = geometry ? cJSON_GetObjectItem(geometry, "coordinates") : NULL;

    if (!cJSON_IsArray(coordinates))
    {
        printf("ROUTE: No route geometry returned.\n");
        cJSON_Delete(root);
        return 0;
    }

    route_point_count = 0;
    route_distance_m = 0.0;
    route_duration_s = 0.0;

    cJSON *coord;
    cJSON_ArrayForEach(coord, coordinates)
    {
        if (route_point_count >= MAX_ROUTE_POINTS)
            break;

        if (!cJSON_IsArray(coord) || cJSON_GetArraySize(coord) < 2)
            continue;

        cJSON *lon_item = cJSON_GetArrayItem(coord, 0);
        cJSON *lat_item = cJSON_GetArrayItem(coord, 1);

        if (!cJSON_IsNumber(lon_item) || !cJSON_IsNumber(lat_item))
            continue;

        route_points[route_point_count].lon = lon_item->valuedouble;
        route_points[route_point_count].lat = lat_item->valuedouble;
        route_point_count++;
    }

    if (route_point_count < 2)
    {
        route_point_count = 0;
        route_valid = 0;
        cJSON_Delete(root);
        printf("ROUTE: Not enough route points.\n");
        return 0;
    }

    route_valid = 1;

    cJSON *distance = route0 ? cJSON_GetObjectItem(route0, "distance") : NULL;
    cJSON *duration = route0 ? cJSON_GetObjectItem(route0, "duration") : NULL;

    if (cJSON_IsNumber(distance) && cJSON_IsNumber(duration))
    {
        route_distance_m = distance->valuedouble;
        route_duration_s = duration->valuedouble;
        printf("ROUTE READY: %.2f km, %.1f min\n",
               route_distance_m / 1000.0,
               route_duration_s / 60.0);
    }
    else
    {
        printf("ROUTE READY: %d points\n", route_point_count);
    }

    double geometry_total = 0.0;
    route_cumulative_m[0] = 0.0;
    for (int i = 1; i < route_point_count; ++i)
    {
        geometry_total += haversine_km(route_points[i - 1].lat, route_points[i - 1].lon,
                                       route_points[i].lat, route_points[i].lon) * 1000.0;
        route_cumulative_m[i] = geometry_total;
    }
    navigation_total_distance_m = route_distance_m > 1.0 ? route_distance_m : geometry_total;
    navigation_total_duration_s = route_duration_s > 1.0 ? route_duration_s :
                                   navigation_total_distance_m / navigation_speed_mps;

    cJSON_Delete(root);
    return 1;
}

static void draw_route_line(void)
{
    clear_route_line();

    if (!map_view || !route_valid || route_point_count < 2)
        return;

    route_draw_points = malloc(sizeof(lv_point_precise_t) * (size_t)route_point_count);
    if (!route_draw_points)
    {
        printf("ROUTE: Could not allocate drawing points.\n");
        return;
    }

    int visible_count = 0;

    for (int i = 0; i < route_point_count; ++i)
    {
        int x, y;
        map_pixel_position(route_points[i].lat,
                           route_points[i].lon,
                           &x, &y);

        /* Keep the route drawable even when it extends slightly outside
           the viewport; LVGL will clip it to map_view. */
        route_draw_points[visible_count].x = x;
        route_draw_points[visible_count].y = y;
        visible_count++;
    }

    if (visible_count < 2)
    {
        free(route_draw_points);
        route_draw_points = NULL;
        return;
    }

    route_line = lv_line_create(map_view);
    lv_line_set_points(route_line,
                       route_draw_points,
                       (uint32_t)visible_count);
    lv_obj_set_style_line_color(route_line,
                                lv_color_hex(0x1A73E8), 0);
    lv_obj_set_style_line_width(route_line, 6, 0);
    lv_obj_set_style_line_rounded(route_line, true, 0);
    lv_obj_clear_flag(route_line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(route_line, LV_OBJ_FLAG_EVENT_BUBBLE);

    /* Put the route behind the markers. */
    lv_obj_move_to_index(route_line, 1);
}

static void map_start_cb(lv_event_t *e)
{
    (void)e;

    if (!destination_valid)
    {
        printf("START: Select a destination first.\n");
        return;
    }

    double lat, lon;
    if (!gps_get_current(&lat, &lon))
    {
        printf("START: Current GPS location unavailable.\n");
        return;
    }

    current_lat = lat;
    current_lon = lon;

    printf("START: Routing from %.6f, %.6f to %.6f, %.6f\n",
           current_lat, current_lon,
           destination_lat, destination_lon);

    if (!request_route())
    {
        printf("START: Could not calculate route.\n");
        return;
    }

    fit_start_destination();
    rebuild_map();

    /* Enter dedicated navigation mode after the route is ready. */
    show_navigation_screen();
}

/* ============================================================
 * MAP SEARCH RESULT SELECTION
 * ============================================================
 */

static void cancel_active_suggestions(void)
{
    if (suggestion_timer)
    {
        lv_timer_del(suggestion_timer);
        suggestion_timer = NULL;
    }

    pthread_mutex_lock(&suggestion_mutex);
    suggestion_generation++;
    pending_ready = 0;
    pthread_mutex_unlock(&suggestion_mutex);

    active_search_input = NULL;
    active_suggestion_box = NULL;
}

static void deferred_select_destination_cb(void *param)
{
    (void)param;

    if (!deferred_selection_pending)
        return;

    deferred_selection_pending = 0;
    cancel_active_suggestions();

    select_destination(deferred_selection.lat,
                       deferred_selection.lon,
                       deferred_selection.name);
}

static void select_destination(double lat, double lon, const char *name)
{
    destination_lat = lat;
    destination_lon = lon;
    destination_valid = 1;
    safe_copy(destination_name, sizeof(destination_name), name);

    /* A newly searched destination needs a fresh route. */
    route_valid = 0;
    route_point_count = 0;
    clear_route_line();

    if (destination_name[0] == '\0')
        safe_copy(destination_name, sizeof(destination_name),
                  "Selected destination");

    if (search_box)
        lv_textarea_set_text(search_box, destination_name);

    if (destination_info)
        lv_label_set_text(destination_info, destination_name);

    map_center_lat = destination_lat;
    map_center_lon = destination_lon;
    map_zoom = 15;
    map_visual_scale = 256;
    show_map();
}

/* Select a map-search suggestion without destroying the current LVGL screen. */
static void select_destination_on_existing_map(double lat, double lon, const char *name)
{
    destination_lat = lat;
    destination_lon = lon;
    destination_valid = 1;
    safe_copy(destination_name, sizeof(destination_name), name);

    /* A newly searched destination needs a fresh route. */
    route_valid = 0;
    route_point_count = 0;
    clear_route_line();

    if (destination_name[0] == '\0')
        safe_copy(destination_name, sizeof(destination_name),
                  "Selected destination");

    /* Keep the selected place in the map search bar. */
    if (map_search_box)
        lv_textarea_set_text(map_search_box, destination_name);

    if (map_suggestions_box)
        lv_obj_add_flag(map_suggestions_box, LV_OBJ_FLAG_HIDDEN);

    /* Invalidate any older suggestion response. */
    cancel_active_suggestions();

    /* Center the map on exactly the selected coordinates. */
    map_center_lat = destination_lat;
    map_center_lon = destination_lon;
    map_zoom = 15;
    map_visual_scale = 256;

    /* Reuse the existing map screen. No lv_obj_clean(), no screen replacement. */
    if (!map_view || !map_image)
        return;

    if (!destination_marker)
        destination_marker = create_destination_marker(map_view);

    if (!destination_name_label)
    {
        destination_name_label = lv_label_create(map_view);
        lv_label_set_long_mode(destination_name_label, LV_LABEL_LONG_DOT);
        lv_obj_set_width(destination_name_label, 260);
        lv_obj_set_style_bg_color(destination_name_label,
                                   lv_color_white(), 0);
        lv_obj_set_style_bg_opa(destination_name_label,
                                 LV_OPA_COVER, 0);
        lv_obj_set_style_radius(destination_name_label, 8, 0);
        lv_obj_set_style_pad_all(destination_name_label, 5, 0);
        lv_obj_clear_flag(destination_name_label, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(destination_name_label, LV_OBJ_FLAG_EVENT_BUBBLE);
    }

    char short_name[240];
    short_place_name(destination_name, short_name, sizeof(short_name));
    lv_label_set_text(destination_name_label, short_name);

    /* Download/render the new map while keeping the same LVGL objects alive. */
    rebuild_map();
    update_all_map_labels();

    printf("\nMAP SEARCH SELECTION\n");
    printf("Name: %s\n", destination_name);
    printf("Lat : %.8f\n", destination_lat);
    printf("Lon : %.8f\n", destination_lon);
}

static void result_button_cb(lv_event_t *e)
{
    int index = (int)(intptr_t)lv_event_get_user_data(e);

    if (index < 0 || index >= result_count)
        return;

    /* Copy the selected result before changing anything in the suggestion box. */
    double lat = results[index].lat;
    double lon = results[index].lon;
    char name[NAME_SIZE];
    safe_copy(name, sizeof(name), results[index].name);

    /*
     * CRITICAL FIX:
     * If the suggestion was clicked on the MAP screen, do NOT destroy/rebuild
     * the LVGL screen from inside the button CLICKED event.  The old code called
     * show_map(), which calls lv_obj_clean(screen).  That deleted the button and
     * its parent while LVGL was still dispatching the click, causing the SDL
     * simulator to close/crash.
     *
     * Instead, update the existing map in place.
     */
    if (map_view && map_suggestions_box &&
        active_suggestion_box == map_suggestions_box)
    {
        select_destination_on_existing_map(lat, lon, name);
        return;
    }

    /* Search-screen selection can safely transition to the map after the event. */
    deferred_selection.lat = lat;
    deferred_selection.lon = lon;
    safe_copy(deferred_selection.name,
              sizeof(deferred_selection.name), name);
    deferred_selection_pending = 1;

    if (results_box)
        lv_obj_add_flag(results_box, LV_OBJ_FLAG_HIDDEN);
    if (map_suggestions_box)
        lv_obj_add_flag(map_suggestions_box, LV_OBJ_FLAG_HIDDEN);

    cancel_active_suggestions();

    /* The async callback runs after the current LVGL event has finished. */
    lv_async_call(deferred_select_destination_cb, NULL);
}


/* ============================================================
 * START BUTTON
 * ============================================================
 */
/*
static void start_button_cb(lv_event_t *e)
{
    (void)e;

    double lat, lon;
    if (!gps_get_current(&lat, &lon))
    {
        show_search_message("GPS location is not available.");
        return;
    }

    current_lat = lat;
    current_lon = lon;

    char place[NAME_SIZE];
    safe_copy(place, sizeof(place), "location");

    if (reverse_geocode(current_lat, current_lon,
                        place, sizeof(place)))
    {
        safe_copy(current_place_name, sizeof(current_place_name), place);
    }
    else
    {
        safe_copy(current_place_name, sizeof(current_place_name),
                  "Current GPS location");
    }

    if (start_info)
        lv_label_set_text(start_info, current_place_name);

    if (destination_valid)
        fit_start_destination();
    else
    {
        map_center_lat = current_lat;
        map_center_lon = current_lon;
        map_zoom = DEFAULT_ZOOM;
        map_visual_scale = 256;
    }

    show_map();
}
*/
/* ============================================================
 * DESTINATION BUTTON
 * ============================================================
 */

static void destination_button_cb(lv_event_t *e)
{
    (void)e;
    attach_keyboard_to_textarea(search_box);

    if (search_box)
        lv_textarea_set_cursor_pos(search_box,
                                   LV_TEXTAREA_CURSOR_LAST);
}

/* ============================================================
 * MAP SEARCH
 * ============================================================
 */

static void map_search_button_cb(lv_event_t *e)
{
    (void)e;

    if (!map_search_box || !map_suggestions_box)
        return;

    const char *text = lv_textarea_get_text(map_search_box);
    if (!text)
        return;

    char query[512];
    safe_copy(query, sizeof(query), text);
    trim_text(query);

    if (strlen(query) < MIN_SEARCH_CHARS)
    {
        clear_suggestion_box(map_suggestions_box);
        lv_obj_add_flag(map_suggestions_box, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    /*
     * IMPORTANT: Do not perform the curl request in this LVGL callback.
     * Repeated searches stay inside the same SDL window because the HTTP
     * operation is performed by the background suggestion worker.
     */
    attach_keyboard_to_textarea(map_search_box);

    pthread_mutex_lock(&suggestion_mutex);
    suggestion_generation++;
    pending_ready = 0;
    pthread_mutex_unlock(&suggestion_mutex);

    if (suggestion_timer)
    {
        lv_timer_del(suggestion_timer);
        suggestion_timer = NULL;
    }

    active_search_input = map_search_box;
    active_suggestion_box = map_suggestions_box;
    lv_obj_clear_flag(map_suggestions_box, LV_OBJ_FLAG_HIDDEN);
    clear_suggestion_box(map_suggestions_box);

    start_suggestion_search(map_search_box, map_suggestions_box);
}

/* ============================================================
 * MAP CLICK -> LAT/LON -> REVERSE GEOCODE
 * ============================================================
 */

static void set_destination_from_map(int screen_x, int screen_y)
{
    if (!map_view)
        return;

    if (screen_x < VIEW_X || screen_x >= VIEW_X + VIEW_W ||
        screen_y < VIEW_Y || screen_y >= VIEW_Y + VIEW_H)
        return;

    /* Convert the screen click into map coordinates. */
    double scale = (double)map_visual_scale / 256.0;

    double image_dx =
        ((double)screen_x - (VIEW_X + VIEW_W / 2.0)) / scale;
    double image_dy =
        ((double)screen_y - (VIEW_Y + VIEW_H / 2.0)) / scale;

    double cx, cy;
    latlon_to_world(map_center_lat, map_center_lon,
                    map_zoom, &cx, &cy);

    double wx = cx + image_dx;
    double wy = cy + image_dy;

    double new_lat, new_lon;
    world_to_latlon(wx, wy, map_zoom, &new_lat, &new_lon);

    /*
     * IMPORTANT:
     * Do NOT perform HTTP/reverse-geocoding here.
     * This function runs from an LVGL mouse event.  A network request
     * from the event handler can make the SDL simulator appear frozen
     * or close when the organization's curl executable is blocked.
     */
    destination_lat = new_lat;
    destination_lon = new_lon;
    destination_valid = 1;

    safe_copy(destination_name, sizeof(destination_name),
              "Selected map location");

    /* Remove only the old marker/label.  Never clean the whole screen. */
    if (destination_marker)
    {
        lv_obj_del(destination_marker);
        destination_marker = NULL;
    }

    if (destination_name_label)
    {
        lv_obj_del(destination_name_label);
        destination_name_label = NULL;
    }

    destination_marker = create_destination_marker(map_view);

    destination_name_label = lv_label_create(map_view);
    lv_label_set_text(destination_name_label, destination_name);
    lv_label_set_long_mode(destination_name_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(destination_name_label, 230);
    lv_obj_set_style_bg_color(destination_name_label,
                               lv_color_white(), 0);
    lv_obj_set_style_bg_opa(destination_name_label,
                             LV_OPA_COVER, 0);
    lv_obj_set_style_radius(destination_name_label, 8, 0);
    lv_obj_set_style_pad_all(destination_name_label, 5, 0);
    lv_obj_clear_flag(destination_name_label, LV_OBJ_FLAG_CLICKABLE);

    update_all_map_labels();

    printf("\\nMAP CLICK DESTINATION\\n");
    printf("Name: %s\\n", destination_name);
    printf("Lat : %.8f\\n", destination_lat);
    printf("Lon : %.8f\\n", destination_lon);
}

/* ============================================================
 * MAP DRAG
 * ============================================================
 */

static void map_press_cb(lv_event_t *e)
{
    (void)e;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev)
        return;

    lv_indev_get_point(indev, &drag_start);
    dragging = 1;
    drag_moved = 0;
}

static void map_pressing_cb(lv_event_t *e)
{
    (void)e;

    if (!dragging)
        return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev)
        return;

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (abs(p.x - drag_start.x) > 5 ||
        abs(p.y - drag_start.y) > 5)
    {
        drag_moved = 1;
        if (navigation_active) navigation_follow_mode = 0;
    }
}

static void map_release_cb(lv_event_t *e)
{
    (void)e;

    if (!dragging)
        return;

    lv_indev_t *indev = lv_indev_get_act();
    if (!indev)
    {
        dragging = 0;
        return;
    }

    lv_point_t end;
    lv_indev_get_point(indev, &end);

    if (drag_moved)
    {
        double cx, cy;
        latlon_to_world(map_center_lat, map_center_lon,
                        map_zoom, &cx, &cy);

        double scale = (double)map_visual_scale / 256.0;

        cx -= (end.x - drag_start.x) / scale;
        cy -= (end.y - drag_start.y) / scale;

        world_to_latlon(cx, cy, map_zoom,
                        &map_center_lat, &map_center_lon);

        rebuild_map();
    }
    else
    {
        /*
         * A simple map click must NOT create/change a destination.
         * The destination pin belongs only to a location selected from
         * the search suggestions.  This prevents a random map click from
         * replacing the searched destination with "Selected map location".
         *
         * Map dragging is still supported above.  A click without movement
         * is intentionally ignored.
         */
    }

    dragging = 0;
}

/* ============================================================
 * ZOOM
 * ============================================================
 */

static void apply_zoom(int direction)
{
    int new_zoom = map_zoom + direction;

    if (new_zoom < MIN_ZOOM || new_zoom > MAX_ZOOM)
        return;

    map_zoom = new_zoom;
    map_visual_scale = 256;

    rebuild_map();
}

static void zoom_in_cb(lv_event_t *e)
{
    (void)e;
    apply_zoom(+1);
}

static void zoom_out_cb(lv_event_t *e)
{
    (void)e;
    apply_zoom(-1);
}

/* ============================================================
 * MY LOCATION
 * ============================================================
 */

static void my_location_cb(lv_event_t *e)
{
    (void)e;

    double lat, lon;
    if (!gps_get_current(&lat, &lon))
        return;

    current_lat = lat;
    current_lon = lon;

    map_center_lat = current_lat;
    map_center_lon = current_lon;
    map_zoom = DEFAULT_ZOOM;
    map_visual_scale = 256;

    rebuild_map();
}

/* ============================================================
 * MAP REBUILD
 * ============================================================
 */

static void rebuild_map(void)
{
    if (!map_view || !map_image)
        return;

    if (!build_map_image())
        return;

    lv_image_set_src(map_image, &map_dsc);
    lv_image_set_pivot(map_image,
                       MAP_IMAGE_SIZE / 2,
                       MAP_IMAGE_SIZE / 2);
    lv_image_set_scale(map_image, 256);
    lv_image_set_antialias(map_image, false);

    /* Center the 768x768 image in the visible map viewport. */
    lv_obj_set_pos(map_image,
                   (VIEW_W - MAP_IMAGE_SIZE) / 2,
                   (VIEW_H - MAP_IMAGE_SIZE) / 2);

    if (route_valid)
        draw_route_line();

    update_all_map_labels();
}

/* ============================================================
 * MAP UI HELPERS
 * ============================================================
 */

static lv_obj_t *make_button(lv_obj_t *parent,
                             const char *text,
                             lv_coord_t w,
                             lv_coord_t h)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_radius(b, h / 2, 0);
    lv_obj_set_style_bg_color(b, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_color(b, lv_color_hex(0xDADCE0), 0);
    lv_obj_set_style_text_color(b, lv_color_hex(0x202124), 0);
    lv_obj_set_style_pad_all(b, 8, 0);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, text ? text : "");
    lv_obj_center(l);
    return b;
}

/* ============================================================
 * GOOGLE-MAPS-STYLE NAVIGATION SCREEN
 * ============================================================
 */

static void stop_navigation_simulation(void)
{
    navigation_active = 0;
    if (navigation_timer) { lv_timer_del(navigation_timer); navigation_timer = NULL; }
}

static int navigation_nearest_route_index(void)
{
    if (!route_valid || route_point_count < 1) return 0;
    int best = navigation_route_index;
    double best_d = 1e30;
    int from = navigation_route_index - 20; if (from < 0) from = 0;
    int to = navigation_route_index + 60; if (to >= route_point_count) to = route_point_count-1;
    for (int i=from;i<=to;i++) {
        double d=haversine_km(current_lat,current_lon,route_points[i].lat,route_points[i].lon);
        if(d<best_d){best_d=d;best=i;}
    }
    return best;
}

static void update_navigation_labels(double remaining_m)
{
    if (navigation_distance_label) {
        char t[64];
        if(remaining_m>=1000.0) snprintf(t,sizeof(t),"%.1f km",remaining_m/1000.0);
        else snprintf(t,sizeof(t),"%.0f m",remaining_m);
        lv_label_set_text(navigation_distance_label,t);
    }
    if (navigation_eta_label) {
        int min=(int)ceil((remaining_m/navigation_speed_mps)/60.0); if(min<1 && remaining_m>0) min=1;
        char t[64];
        if(min>=60) snprintf(t,sizeof(t),"%dh %02d min",min/60,min%60);
        else snprintf(t,sizeof(t),"%d min",min);
        lv_label_set_text(navigation_eta_label,t);
    }
}

static void navigation_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if(!navigation_active || !route_valid || route_point_count<2) return;
    double step=navigation_speed_mps;
    while(step>0 && navigation_route_index<route_point_count-1) {
        double seg=haversine_km(route_points[navigation_route_index].lat,route_points[navigation_route_index].lon,
                                route_points[navigation_route_index+1].lat,route_points[navigation_route_index+1].lon)*1000.0;
        if(seg<0.5){navigation_route_index++;navigation_segment_progress_m=0;continue;}
        double left=seg-navigation_segment_progress_m;
        if(step<left){navigation_segment_progress_m+=step;step=0;}
        else{step-=left;navigation_route_index++;navigation_segment_progress_m=0;}
    }
    if(navigation_route_index>=route_point_count-1){
        current_lat=destination_lat; current_lon=destination_lon;
        update_navigation_labels(0);
        if(navigation_instruction) lv_label_set_text(navigation_instruction,"You have arrived");
        stop_navigation_simulation();
        if(navigation_follow_mode){map_center_lat=current_lat;map_center_lon=current_lon;rebuild_map();}
        return;
    }
    double seg=haversine_km(route_points[navigation_route_index].lat,route_points[navigation_route_index].lon,
                            route_points[navigation_route_index+1].lat,route_points[navigation_route_index+1].lon)*1000.0;
    double f=seg>0.5?navigation_segment_progress_m/seg:0; if(f<0)f=0;if(f>1)f=1;
    current_lat=route_points[navigation_route_index].lat+(route_points[navigation_route_index+1].lat-route_points[navigation_route_index].lat)*f;
    current_lon=route_points[navigation_route_index].lon+(route_points[navigation_route_index+1].lon-route_points[navigation_route_index].lon)*f;
    double geom=route_cumulative_m[route_point_count-1];
    double progressed=route_cumulative_m[navigation_route_index]+navigation_segment_progress_m;
    double remaining=navigation_total_distance_m*(geom>1?fmax(0.0,1.0-progressed/geom):0.0);
    update_navigation_labels(remaining);
    if(navigation_instruction) lv_label_set_text(navigation_instruction,"Follow the highlighted route");
    if(navigation_follow_mode){map_center_lat=current_lat;map_center_lon=current_lon;rebuild_map();}
    else update_all_map_labels();
}

static void navigation_stop_cb(lv_event_t *e)
{
    (void)e; stop_navigation_simulation(); navigation_follow_mode=1; show_map();
}

static void show_navigation_screen(void)
{
    app_page = 2;
    lv_obj_t *screen=lv_screen_active();
    stop_navigation_simulation(); cancel_active_suggestions(); clear_route_line(); lv_obj_clean(screen);
    if(!build_map_image()) printf("NAVIGATION: map build failed.\n");

    map_view=lv_obj_create(screen);
    lv_obj_set_size(map_view,VIEW_W,VIEW_H); lv_obj_set_pos(map_view,VIEW_X,VIEW_Y);
    lv_obj_set_style_pad_all(map_view,0,0); lv_obj_set_style_border_width(map_view,0,0); lv_obj_clear_flag(map_view,LV_OBJ_FLAG_SCROLLABLE);
    map_image=lv_image_create(map_view); lv_image_set_src(map_image,&map_dsc);
    lv_image_set_pivot(map_image,MAP_IMAGE_SIZE/2,MAP_IMAGE_SIZE/2); lv_image_set_scale(map_image,256); lv_image_set_antialias(map_image,false);
    lv_obj_set_pos(map_image,(VIEW_W-MAP_IMAGE_SIZE)/2,(VIEW_H-MAP_IMAGE_SIZE)/2);
    lv_obj_add_flag(map_image,LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(map_image,map_press_cb,LV_EVENT_PRESSED,NULL); lv_obj_add_event_cb(map_image,map_pressing_cb,LV_EVENT_PRESSING,NULL); lv_obj_add_event_cb(map_image,map_release_cb,LV_EVENT_RELEASED,NULL);
    current_marker=create_current_marker(map_view); destination_marker=destination_valid?create_destination_marker(map_view):NULL; current_name_label=NULL;
    if(destination_valid){destination_name_label=lv_label_create(map_view);char n[240];short_place_name(destination_name,n,sizeof(n));lv_label_set_text(destination_name_label,n);lv_obj_set_width(destination_name_label,250);lv_obj_set_style_bg_color(destination_name_label,lv_color_white(),0);lv_obj_set_style_bg_opa(destination_name_label,LV_OPA_COVER,0);lv_obj_set_style_radius(destination_name_label,10,0);lv_obj_set_style_border_width(destination_name_label,1,0);lv_obj_set_style_border_color(destination_name_label,lv_color_hex(0xDADCE0),0);lv_obj_set_style_pad_all(destination_name_label,5,0);}else destination_name_label=NULL;
    draw_route_line(); update_all_map_labels();

    lv_obj_t *top=lv_obj_create(screen); lv_obj_set_size(top,500,92); lv_obj_set_pos(top,72,16);
    lv_obj_set_style_bg_color(top,lv_color_white(),0); lv_obj_set_style_bg_opa(top,LV_OPA_COVER,0); lv_obj_set_style_radius(top,18,0); lv_obj_set_style_border_width(top,1,0); lv_obj_set_style_border_color(top,lv_color_hex(0xE0E0E0),0); lv_obj_set_style_pad_all(top,12,0);
    navigation_title=lv_label_create(top);lv_label_set_text(navigation_title,"NAVIGATION");lv_obj_set_style_text_color(navigation_title,lv_color_hex(0x5F6368),0);lv_obj_align(navigation_title,LV_ALIGN_TOP_LEFT,8,0);
    navigation_instruction=lv_label_create(top);lv_label_set_text(navigation_instruction,"Follow the highlighted route");lv_obj_set_width(navigation_instruction,455);lv_obj_align(navigation_instruction,LV_ALIGN_BOTTOM_LEFT,8,-6);
    navigation_back_button=make_button(screen,"<",50,50);lv_obj_set_pos(navigation_back_button,14,16);lv_obj_add_event_cb(navigation_back_button,back_cb,LV_EVENT_CLICKED,NULL);

    lv_obj_t *info=lv_obj_create(screen);lv_obj_set_size(info,620,112);lv_obj_set_pos(info,90,472);lv_obj_set_style_bg_color(info,lv_color_white(),0);lv_obj_set_style_bg_opa(info,LV_OPA_COVER,0);lv_obj_set_style_radius(info,20,0);lv_obj_set_style_border_width(info,1,0);lv_obj_set_style_border_color(info,lv_color_hex(0xE0E0E0),0);lv_obj_set_style_pad_all(info,12,0);
    navigation_destination_label=lv_label_create(info);lv_label_set_text(navigation_destination_label,destination_name);lv_label_set_long_mode(navigation_destination_label,LV_LABEL_LONG_DOT);lv_obj_set_width(navigation_destination_label,390);lv_obj_align(navigation_destination_label,LV_ALIGN_TOP_LEFT,8,2);
    navigation_distance_label=lv_label_create(info);lv_obj_align(navigation_distance_label,LV_ALIGN_BOTTOM_LEFT,8,-6);
    navigation_eta_label=lv_label_create(info);lv_obj_align(navigation_eta_label,LV_ALIGN_BOTTOM_LEFT,105,-6);
    navigation_stop_button=make_button(screen,"STOP",90,48);lv_obj_set_pos(navigation_stop_button,600,500);lv_obj_add_event_cb(navigation_stop_button,navigation_stop_cb,LV_EVENT_CLICKED,NULL);

    lv_obj_t *loc=make_button(screen,"◎",50,50);lv_obj_set_pos(loc,738,370);lv_obj_add_event_cb(loc,my_location_cb,LV_EVENT_CLICKED,NULL);
    zoom_in_button=make_button(screen,"+",46,46);lv_obj_set_pos(zoom_in_button,739,426);lv_obj_add_event_cb(zoom_in_button,zoom_in_cb,LV_EVENT_CLICKED,NULL);
    zoom_out_button=make_button(screen,"-",46,46);lv_obj_set_pos(zoom_out_button,739,478);lv_obj_add_event_cb(zoom_out_button,zoom_out_cb,LV_EVENT_CLICKED,NULL);
    map_attribution=lv_label_create(screen);lv_label_set_text(map_attribution,"© OpenStreetMap contributors");lv_obj_set_style_text_color(map_attribution,lv_color_hex(0x5F6368),0);lv_obj_set_style_bg_color(map_attribution,lv_color_white(),0);lv_obj_set_style_bg_opa(map_attribution,LV_OPA_COVER,0);lv_obj_set_style_radius(map_attribution,5,0);lv_obj_align(map_attribution,LV_ALIGN_BOTTOM_RIGHT,-8,-4);

    navigation_follow_mode=1; navigation_active=1; navigation_route_index=navigation_nearest_route_index(); navigation_segment_progress_m=0; update_navigation_labels(navigation_total_distance_m); navigation_timer=lv_timer_create(navigation_timer_cb,1000,NULL);
}

static void show_map(void)
{
    app_page = 1;
    lv_obj_t *screen=lv_screen_active();
    stop_navigation_simulation(); cancel_active_suggestions(); lv_obj_clean(screen);
    if(!build_map_image()) printf("MAP ERROR: could not build map image.\n");

    map_view=lv_obj_create(screen);lv_obj_set_size(map_view,VIEW_W,VIEW_H);lv_obj_set_pos(map_view,VIEW_X,VIEW_Y);lv_obj_set_style_pad_all(map_view,0,0);lv_obj_set_style_border_width(map_view,0,0);lv_obj_clear_flag(map_view,LV_OBJ_FLAG_SCROLLABLE);
    map_image=lv_image_create(map_view);lv_image_set_src(map_image,&map_dsc);lv_image_set_pivot(map_image,MAP_IMAGE_SIZE/2,MAP_IMAGE_SIZE/2);lv_image_set_scale(map_image,256);lv_image_set_antialias(map_image,false);lv_obj_set_pos(map_image,(VIEW_W-MAP_IMAGE_SIZE)/2,(VIEW_H-MAP_IMAGE_SIZE)/2);
    lv_obj_add_flag(map_view,LV_OBJ_FLAG_CLICKABLE);lv_obj_add_event_cb(map_view,map_press_cb,LV_EVENT_PRESSED,NULL);lv_obj_add_event_cb(map_view,map_pressing_cb,LV_EVENT_PRESSING,NULL);lv_obj_add_event_cb(map_view,map_release_cb,LV_EVENT_RELEASED,NULL);
    current_marker=create_current_marker(map_view);destination_marker=destination_valid?create_destination_marker(map_view):NULL;current_name_label=NULL;destination_name_label=NULL;
    if(destination_valid){destination_name_label=lv_label_create(map_view);char n[240];short_place_name(destination_name,n,sizeof(n));lv_label_set_text(destination_name_label,n);lv_label_set_long_mode(destination_name_label,LV_LABEL_LONG_DOT);lv_obj_set_width(destination_name_label,250);lv_obj_set_style_bg_color(destination_name_label,lv_color_white(),0);lv_obj_set_style_bg_opa(destination_name_label,LV_OPA_COVER,0);lv_obj_set_style_radius(destination_name_label,10,0);lv_obj_set_style_border_width(destination_name_label,1,0);lv_obj_set_style_border_color(destination_name_label,lv_color_hex(0xDADCE0),0);lv_obj_set_style_pad_all(destination_name_label,5,0);}
    draw_route_line();update_all_map_labels();

    lv_obj_t *search_card=lv_obj_create(screen);lv_obj_set_size(search_card,500,58);lv_obj_set_pos(search_card,16,16);lv_obj_set_style_bg_color(search_card,lv_color_white(),0);lv_obj_set_style_bg_opa(search_card,LV_OPA_COVER,0);lv_obj_set_style_radius(search_card,18,0);lv_obj_set_style_border_width(search_card,1,0);lv_obj_set_style_border_color(search_card,lv_color_hex(0xDADCE0),0);lv_obj_set_style_pad_all(search_card,0,0);lv_obj_clear_flag(search_card,LV_OBJ_FLAG_CLICKABLE);
    back_button=make_button(screen,"<",48,48);lv_obj_set_pos(back_button,24,21);lv_obj_add_event_cb(back_button,back_cb,LV_EVENT_CLICKED,NULL);
    map_search_box=lv_textarea_create(screen);lv_textarea_set_one_line(map_search_box,true);lv_textarea_set_placeholder_text(map_search_box,"Search places...");lv_obj_set_size(map_search_box,320,48);lv_obj_set_pos(map_search_box,78,21);style_search_box(map_search_box);lv_obj_set_style_border_width(map_search_box,0,0);lv_obj_add_event_cb(map_search_box,search_input_value_changed_cb,LV_EVENT_VALUE_CHANGED,NULL);
    map_search_button=make_button(screen,"SEARCH",96,44);lv_obj_set_pos(map_search_button,406,23);lv_obj_add_event_cb(map_search_button,map_search_button_cb,LV_EVENT_CLICKED,NULL);
    map_suggestions_box=lv_obj_create(screen);lv_obj_set_size(map_suggestions_box,500,300);lv_obj_set_pos(map_suggestions_box,16,78);lv_obj_set_style_bg_color(map_suggestions_box,lv_color_white(),0);lv_obj_set_style_bg_opa(map_suggestions_box,LV_OPA_COVER,0);lv_obj_set_style_radius(map_suggestions_box,14,0);lv_obj_set_style_border_width(map_suggestions_box,1,0);lv_obj_set_style_border_color(map_suggestions_box,lv_color_hex(0xE0E0E0),0);lv_obj_set_style_pad_all(map_suggestions_box,8,0);lv_obj_set_flex_flow(map_suggestions_box,LV_FLEX_FLOW_COLUMN);lv_obj_set_scroll_dir(map_suggestions_box,LV_DIR_VER);lv_obj_add_flag(map_suggestions_box,LV_OBJ_FLAG_HIDDEN);attach_keyboard_to_textarea(map_search_box);

    /* Google-Maps-style top controls: START + CURRENT.
     * Category chips are intentionally removed so the map stays clean. */
    map_start_button=make_button(screen,"START",94,46);
    lv_obj_set_pos(map_start_button,580,20);
    lv_obj_set_style_bg_color(map_start_button,lv_color_hex(0x1A73E8),0);
    lv_obj_set_style_text_color(map_start_button,lv_color_white(),0);
    lv_obj_set_style_border_width(map_start_button,0,0);
    lv_obj_add_event_cb(map_start_button,map_start_cb,LV_EVENT_CLICKED,NULL);

    my_location_button=make_button(screen,"CURRENT",108,46);
    lv_obj_set_pos(my_location_button,684,20);
    lv_obj_set_style_bg_color(my_location_button,lv_color_white(),0);
    lv_obj_set_style_text_color(my_location_button,lv_color_hex(0x202124),0);
    lv_obj_set_style_border_width(my_location_button,1,0);
    lv_obj_set_style_border_color(my_location_button,lv_color_hex(0xDADCE0),0);
    lv_obj_add_event_cb(my_location_button,my_location_cb,LV_EVENT_CLICKED,NULL);

    zoom_in_button=make_button(screen,"+",46,46);
    lv_obj_set_pos(zoom_in_button,739,410);
    lv_obj_add_event_cb(zoom_in_button,zoom_in_cb,LV_EVENT_CLICKED,NULL);
    zoom_out_button=make_button(screen,"-",46,46);lv_obj_set_pos(zoom_out_button,739,462);lv_obj_add_event_cb(zoom_out_button,zoom_out_cb,LV_EVENT_CLICKED,NULL);
    map_attribution=lv_label_create(screen);lv_label_set_text(map_attribution,"© OpenStreetMap contributors");lv_obj_set_style_text_color(map_attribution,lv_color_hex(0x5F6368),0);lv_obj_set_style_bg_color(map_attribution,lv_color_white(),0);lv_obj_set_style_bg_opa(map_attribution,LV_OPA_COVER,0);lv_obj_set_style_radius(map_attribution,5,0);lv_obj_align(map_attribution,LV_ALIGN_BOTTOM_RIGHT,-8,-4);
}

/* ============================================================
 * BACK TO SEARCH SCREEN
 * ============================================================
 */

static void back_cb(lv_event_t *e)
{
    (void)e;

    /*
     * IMPORTANT: this is navigation inside the LVGL simulator.
     * Never terminate the SDL/LVGL application here.
     *
     * Navigation screen -> return to map.
     * Map screen         -> return to search screen.
     */
    cancel_active_suggestions();

    if (app_page == 2)
    {
        /* Keep destination and calculated route when returning to the map. */
        stop_navigation_simulation();
        navigation_follow_mode = 1;
        show_map();
        return;
    }

    if (app_page == 1)
    {
        /* Leaving the map does not close the application. */
        stop_navigation_simulation();
        clear_route_line();
        if (keyboard_group)
        {
            lv_group_delete(keyboard_group);
            keyboard_group = NULL;
        }
        create_search_screen();
        return;
    }

    /* Already on the first screen: do nothing. */
}

/* ============================================================
 * SEARCH SCREEN
 * ============================================================
 */

static void map_navigation_button_cb(lv_event_t *e)
{
    (void)e;

    /*
     * MAP NAVIGATION opens the map inside the same LVGL/SDL simulator.
     * If a destination has already been selected and a route exists,
     * go directly to the navigation screen. Otherwise open the map
     * so the user can search/select a destination first.
     */
    if (destination_valid && route_valid)
        show_navigation_screen();
    else
        show_map();
}

static void create_search_screen(void)
{
    app_page = 0;
    lv_obj_t *screen=lv_screen_active();lv_obj_clean(screen);lv_obj_set_style_bg_color(screen,lv_color_hex(0xF8F9FA),0);
    lv_obj_t *title=lv_label_create(screen);lv_label_set_text(title,"MAP NAVIGATION");lv_obj_set_style_text_color(title,lv_color_hex(0x202124),0);lv_obj_align(title,LV_ALIGN_TOP_MID,0,35);
    lv_obj_t *sub=lv_label_create(screen);lv_label_set_text(sub,"OpenStreetMap  •  Real road routing  •  Worldwide search");lv_obj_set_style_text_color(sub,lv_color_hex(0x5F6368),0);lv_obj_align(sub,LV_ALIGN_TOP_MID,0,65);
    lv_obj_t *card=lv_obj_create(screen);lv_obj_set_size(card,650,170);lv_obj_set_pos(card,75,120);lv_obj_set_style_bg_color(card,lv_color_white(),0);lv_obj_set_style_bg_opa(card,LV_OPA_COVER,0);lv_obj_set_style_radius(card,20,0);lv_obj_set_style_border_width(card,1,0);lv_obj_set_style_border_color(card,lv_color_hex(0xE0E0E0),0);
    search_box=lv_textarea_create(card);lv_textarea_set_one_line(search_box,true);lv_textarea_set_placeholder_text(search_box,"Search a place, city, landmark...");lv_obj_set_size(search_box,470,50);lv_obj_set_pos(search_box,18,20);style_search_box(search_box);lv_obj_add_event_cb(search_box,search_input_value_changed_cb,LV_EVENT_VALUE_CHANGED,NULL);
    lv_obj_t *sb=make_button(card,"SEARCH",110,48);lv_obj_set_pos(sb,510,21);lv_obj_add_event_cb(sb,search_button_event_cb,LV_EVENT_CLICKED,NULL);
    destination_info=lv_label_create(card);lv_label_set_text(destination_info,"Select a suggestion to set the destination.");lv_obj_set_style_text_color(destination_info,lv_color_hex(0x5F6368),0);lv_obj_set_width(destination_info,590);lv_obj_align(destination_info,LV_ALIGN_BOTTOM_LEFT,18,-18);
    results_box=lv_obj_create(screen);lv_obj_set_size(results_box,650,235);lv_obj_set_pos(results_box,75,305);lv_obj_set_style_bg_color(results_box,lv_color_white(),0);lv_obj_set_style_bg_opa(results_box,LV_OPA_COVER,0);lv_obj_set_style_radius(results_box,16,0);lv_obj_set_style_border_width(results_box,1,0);lv_obj_set_style_border_color(results_box,lv_color_hex(0xE0E0E0),0);lv_obj_set_style_pad_all(results_box,10,0);lv_obj_set_flex_flow(results_box,LV_FLEX_FLOW_COLUMN);lv_obj_set_scroll_dir(results_box,LV_DIR_VER);
    lv_obj_t *mb=make_button(screen,"MAP NAVIGATION",250,52);lv_obj_align(mb,LV_ALIGN_BOTTOM_MID,0,-12);lv_obj_set_style_bg_color(mb,lv_color_hex(0x1A73E8),0);lv_obj_set_style_text_color(mb,lv_color_white(),0);lv_obj_set_style_border_width(mb,0,0);lv_obj_add_event_cb(mb,map_navigation_button_cb,LV_EVENT_CLICKED,NULL);
    if(destination_valid)lv_label_set_text(destination_info,destination_name);
    show_search_message("Type at least 2 characters to search.");attach_keyboard_to_textarea(search_box);
}

/* ============================================================
 * MAIN
 * ============================================================
 */

int main(void)
{
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK)
    {
        printf("ERROR: libcurl initialization failed.\n");
        return 1;
    }

    lv_init();
    printf("LVGL initialized successfully!\n");

    lv_display_t *display = lv_sdl_window_create(
        WINDOW_WIDTH, WINDOW_HEIGHT);

    if (!display)
    {
        printf("ERROR: SDL window creation failed.\n");
        return 1;
    }

    lv_sdl_mouse_create();
    keyboard_indev = lv_sdl_keyboard_create();

    create_search_screen();

    /* Poll HTTP results only from the LVGL thread. */
    suggestion_poll_timer = lv_timer_create(suggestion_poll_cb, 100, NULL);

    printf("\n============================================\n");
    printf("       OSM LVGL NAVIGATION SIMULATOR\n");
    printf("============================================\n");

    printf("DESTINATION : Nominatim search\n");
    printf("SEARCH      : any location worldwide\n");
    printf("MAP         : OpenStreetMap tiles\n");
    printf("PIN NAME    : OSM reverse geocoding\n");
    printf("PAN         : enabled\n");
    printf("ZOOM        : enabled\n");
    printf("============================================\n");
    while (1)
    {
        lv_timer_handler();
        lv_delay_ms(5);
    }

    return 0;
}
