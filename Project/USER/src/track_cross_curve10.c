/* Algorithm 10 is an isolated CrossCurve FSM instance with a faster,
 * path-supported ring exit handoff. Algorithm 9 remains unchanged for A/B. */
#define TRACK_CROSS_CURVE_VARIANT10 1
#define track_cross_curve_rows track_cross_curve10_rows
#define track_cross_curve_path_x track_cross_curve10_path_x
#define track_cross_curve_path_confidence track_cross_curve10_path_confidence
#define track_cross_curve_reset track_cross_curve10_reset
#define track_cross_curve_suspend track_cross_curve10_suspend
#define track_cross_curve_process track_cross_curve10_process
#define track_cross_curve_binary_preview track_cross_curve10_binary_preview
#define track_cross_curve_state_name track_cross_curve10_state_name
#include "track_cross_curve.c"
