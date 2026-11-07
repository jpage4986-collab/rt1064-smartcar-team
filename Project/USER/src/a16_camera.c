/* Reproject the measured camera to the ground-plane coordinates used by A16.
 * The table is built once. An index outside the sensor has no road evidence. */
#include "a16_internal.h"
#if TRACK_ALGORITHM == 16
#include <math.h>

static uint16_t source_pixel[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];
static uint8_t normalized[TRACK_MODE1_HEIGHT][TRACK_MODE1_WIDTH];

static void build_camera_map(void)
{
    const float focal = 122.5f;
    const float pitch = TRACK_A16_CAMERA_PITCH_DEG * 0.01745329252f;
    const float cs = cosf(pitch), sn = sinf(pitch);
    for (int x = 0; x < TRACK_MODE1_WIDTH; ++x) {
        float u = (x - 93.5f) / focal;
        float c = 1.0f / sqrtf(1.0f + u*u);
        for (int y = 0; y < TRACK_MODE1_HEIGHT; ++y) {
            float v = (59.5f - y) / focal;
            float down = c * 0.258819f - v * 0.965926f;
            float forward = 0.23f * (c * 0.965926f + v * 0.258819f);
            float lateral = 0.23f * u * c;
            float height = TRACK_A16_CAMERA_HEIGHT_M * down;
            float depth = forward * cs + height * sn;
            float up = forward * sn - height * cs;
            int col = (int)floorf(94.0f + focal * lateral / depth);
            int row = (int)floorf(60.0f - focal * up / sqrtf(depth*depth + lateral*lateral));
            source_pixel[y][x] = down > 0.f && (unsigned)col < TRACK_MODE1_WIDTH &&
                (unsigned)row < TRACK_MODE1_HEIGHT ? row * TRACK_MODE1_WIDTH + col : UINT16_MAX;
        }
    }
}

const uint8_t (*a16_reference_view(const uint8_t raw[][TRACK_MODE1_WIDTH]))[TRACK_MODE1_WIDTH]
{
    static int initialized;
    if (!initialized) {
        build_camera_map();
        initialized = 1;
    }
    const uint8_t *pixels = (const uint8_t *)raw;
    for (int row = 0; row < TRACK_MODE1_HEIGHT; ++row) {
        for (int col = 0; col < TRACK_MODE1_WIDTH; ++col) {
            uint16_t source = source_pixel[row][col];
            normalized[row][col] = source == UINT16_MAX ? 0U : pixels[source];
        }
    }
    return normalized;
}
#endif
